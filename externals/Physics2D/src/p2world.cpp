/*
MIT License

Copyright (c) 2017 SAE Institute Switzerland AG

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#define _CRTDBG_MAP_ALLOC
#include<iostream>
#include <crtdbg.h>
#ifdef _DEBUG
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

#include <p2world.h>
#include <p2body.h>
#include <p2contact.h>

#include <cstdint>
#include <iostream>

p2World::p2World(p2Vec2 gravity)
{
	m_Gravity = gravity;
}

p2World::~p2World()
{
	auto it = m_BodyList.begin();

	m_ContactManager.Destroy();

	while (it != m_BodyList.end()) {
		delete(*it);
		it = m_BodyList.erase(it);
	}
}

void p2World::Step(float dt)
{
	for (auto body : m_BodyList) {
		//Compute force (Currently only gravity on dynamic object)
		p2Vec2 forces;
		if (body->GetType() == p2BodyType::DYNAMIC) {
			body->AddForce(m_Gravity * body->GetGravityScale());
		}

		//Compute new velocities
		body->SetLinearVelocity((body->GetForce() * dt) + body->GetLinearVelocity());

		//Set position and angle according to current linear and angular velocities
		body->SetPosition((body->GetLinearVelocity() * dt) + body->GetPosition());
		body->SetAngle(body->GetAngularVelocity() * dt + body->GetAngle());
	}

	//Find new Contact
	m_ContactManager.FindNewContact(m_BodyList);

	//Solve collision
	m_ContactManager.Solve();

	//Check existing contact AABB, and remove those who are not colliding anymore
	m_ContactManager.Collide();

	//Clear all force
	for (p2Body* body : m_BodyList)
	{
		body->ClearForce();
	}
}

p2Body * p2World::CreateBody(p2BodyDef* bodyDef)
{
	p2Body* tmpBody = new p2Body(*bodyDef, this);
	m_BodyList.push_front(tmpBody);

	return *m_BodyList.begin();
}

void p2World::SetContactListener(p2ContactListener * contactListener)
{
	m_ContactManager.SetContactListener(contactListener);
}

p2Vec2 p2World::GetGravity() const
{
	return m_Gravity;
}

std::list<p2Body*> p2World::AABBOverlap(p2AABB aabb)
{
	return m_ContactManager.GetQuadTree()->AABBOverlap(aabb);
}

std::list<p2Body*> p2World::CircleOverlap(p2AABB aabb)
{
	//Get circle information from aabb  (assuming it's a square)
	p2Vec2 center = aabb.GetCenter();
	float radius = aabb.topRight.x - center.x;

	//Get all bodies from the aabb
	std::list<p2Body*> bodies = AABBOverlap(aabb);

	//Remove all bodies not inside the circle
	auto it = bodies.begin();
	while (it != bodies.end())
	{
		p2Vec2 bodyPosition = (*it)->GetPosition();
		float distance = (bodyPosition.x - center.x) * (bodyPosition.x - center.x) + (bodyPosition.y - center.y) * (bodyPosition.y - center.y);

		if (distance > radius * radius) {
			bodies.erase(it++);
		}
		else {
			++it;
		}
	}

	return bodies;
}

std::list<p2Body*> p2World::RaycastAll(p2Vec2 vector, p2Vec2 position, float maxDistance)
{
	p2Vec2 posB = position + vector.Normalized() * maxDistance;

	//Create aabb
	p2AABB aabb;

	aabb.bottomLeft.x = std::fmin(position.x, posB.x);
	aabb.bottomLeft.y = std::fmax(position.y, posB.y);

	aabb.topRight.x = std::fmax(position.x, posB.x);
	aabb.topRight.y = std::fmin(position.y, posB.y);

	//Get all possible contact
	std::list<p2Body*> bodies = AABBOverlap(aabb);

	p2Body body;

	//Create collider used as raycast
	p2ColliderDef colliderDef;
	p2LineShape* line = new p2LineShape();
	line->posA = position;
	line->posB = posB;
	colliderDef.shape = line;
	p2Collider* lineCollider =  new p2Collider(colliderDef, &body);

	//Remove all bodies not collidings with the raycast
	auto it = bodies.begin();
	while (it != bodies.end()) {
		bool isTouching = false;

		for (p2Collider* collider : (*it)->GetColliders()) {

			p2Manifold manifold;

			p2Contact tmp = p2Contact(collider, lineCollider);
			if (collider->GetShapeType() == p2ColliderDef::ShapeType::CIRCLE) {
				if (SAT::CheckCollisionLineCircle(&tmp, manifold)) {
					isTouching = true;
				}
			}

			if (collider->GetShapeType() == p2ColliderDef::ShapeType::POLYGON) {
				if (SAT::CheckCollisionLinePolygon(&tmp, manifold)) {
					isTouching = true;
				}
			}
		}

		if (!isTouching) {
			bodies.erase(it++);
		}
		else {
			it++;
		}
	}

	//Draw raycast if needed
	uint32_t flags = m_DebugDraw->GetFlags();

	if (flags && p2Draw::raycastBit) {
		raycastStruct tmp;
		tmp.posA = position;
		tmp.posB = posB;
		m_DebugDraw->m_Segment.push_front(tmp);
	}

	delete(line);
	delete(lineCollider);

	return std::list<p2Body*>();
}

p2Body * p2World::Raycast(p2Vec2 vector, p2Vec2 position, float maxDistance)
{
	p2Vec2 posB = position + vector.Normalized() * maxDistance;

	p2Body* closestBody = nullptr;
	p2Vec2 contactpoint = posB;
	float minDistance = std::numeric_limits<float>::infinity();

	//Create aabb
	p2AABB aabb;

	aabb.bottomLeft.x = std::fmin(position.x, posB.x);
	aabb.bottomLeft.y = std::fmax(position.y, posB.y);

	aabb.topRight.x = std::fmax(position.x, posB.x);
	aabb.topRight.y = std::fmin(position.y, posB.y);

	//Get all possible contact
	std::list<p2Body*> bodies = AABBOverlap(aabb);

	p2Body body;

	//Create collider used as raycast
	p2ColliderDef colliderDef;
	p2LineShape* line = new p2LineShape();
	line->posA = position;
	line->posB = position + vector.Normalized() * maxDistance;
	colliderDef.shape = line;
	p2Collider* lineCollider = new p2Collider(colliderDef, &body);

	//Remove all non colliding bodies
	auto it = bodies.begin();
	while (it != bodies.end()) {
		bool isTouching = false;

		for (p2Collider* collider : (*it)->GetColliders()) {

			p2Manifold manifold;

			p2Contact tmp = p2Contact(collider, lineCollider);
			if (collider->GetShapeType() == p2ColliderDef::ShapeType::CIRCLE) {
				if (SAT::CheckCollisionLineCircle(&tmp, manifold)) {

					float distance = (line->posA - manifold.contactPoint).GetMagnitude();
					if (minDistance > distance) {
						minDistance = distance;
						closestBody = *it;
						contactpoint = manifold.contactPoint;
						isTouching = true;
					}
				}
			}

			if (collider->GetShapeType() == p2ColliderDef::ShapeType::POLYGON) {
				if (SAT::CheckCollisionLinePolygon(&tmp, manifold)) {
					float distance = (line->posA - manifold.contactPoint).GetMagnitude();
					if (minDistance > distance) {
						minDistance = distance;
						closestBody = *it;
						contactpoint = manifold.contactPoint;
						isTouching = true;
					}
				}
			}
		}

		if (!isTouching) {
			bodies.erase(it++);
		}
		else {
			it++;
		}
	}

	//Draw raycast if needed
	uint32_t flags = m_DebugDraw->GetFlags();

	if (flags && p2Draw::raycastBit) {
		raycastStruct tmp;
		tmp.posA = position;
		if (closestBody == nullptr) {
			tmp.posB = posB;
		}
		else {
			tmp.posB = contactpoint;
		}
		m_DebugDraw->m_Segment.push_front(tmp);
	}

	delete(line);
	delete(lineCollider);

	return closestBody;
}

void p2World::SetDebugDraw(p2Draw * debugDraw)
{
	m_DebugDraw = debugDraw;
}

void p2World::DrawDebugData()
{
	if (m_DebugDraw == nullptr) {
		return;
	}
	
	uint32_t flags = m_DebugDraw->GetFlags();

	//Debug draw quad tree
	m_ContactManager.GetQuadTree()->Draw(m_DebugDraw);

	//Draw vector for sat
	m_ContactManager.Draw(m_DebugDraw);

	for (p2Body* body : m_BodyList) {
		//Draw aabb of body
		if (flags & p2Draw::aabbBit) {
			m_DebugDraw->DrawRect(body->GetAABB().bottomLeft, 0, body->GetAABB().GetExtends() * 2, p2Color(153, 0, 0));
		}

		//Draw center of mass
		if (flags & p2Draw::centerOfMassBit) {
			m_DebugDraw->DrawCircleFilled(body->GetCentroide(), 0.05, p2Color(0, 0, 153));
		}

		for (p2Collider* collider : body->GetColliders()) {
			//Draw collider
			if (flags & p2Draw::colliderBit) {
				if (collider->GetShapeType() == p2ColliderDef::ShapeType::CIRCLE){
					p2CircleShape *circle = static_cast<p2CircleShape*>(collider->GetShape());

					p2Vec2 center = collider->GetPosition();
					float radius = circle->GetRadius();

					m_DebugDraw->DrawCircleFilled(center, radius, p2Color(0, 0, 153));
					
				}

				if (collider->GetShapeType() == p2ColliderDef::ShapeType::POLYGON) {
					p2PolygonShape *polygon = static_cast<p2PolygonShape*>(collider->GetShape());

					m_DebugDraw->DrawPolygonFilled(polygon->GetVerticesWorld(collider->GetPosition(), body->GetAngle()), p2Color(0, 0, 153));
				}
			}

			//Draw aabb of collider
			if (flags & p2Draw::aabbColliderBit) {
				m_DebugDraw->DrawRect(collider->aabb.bottomLeft, 0, collider->aabb.GetExtends() * 2, p2Color(153, 0, 0));
			}
		}
	}


	//Draw raycast
	for (auto it = m_DebugDraw->m_Segment.begin(); it != m_DebugDraw->m_Segment.end(); ) {
		m_DebugDraw->DrawLine(it->posA, it->posB);

		it = m_DebugDraw->m_Segment.erase(it);
	}
}

p2Draw * p2World::GetDebugDraw() const
{
	return m_DebugDraw;
}
