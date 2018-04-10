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
#include <p2world.h>
#include <p2body.h>

#include <cstdint>
#include <iostream>

p2World::p2World(p2Vec2 gravity)
{
	m_Gravity = gravity;
}

p2World::~p2World()
{
	for each (p2Body* body in m_BodyList)
	{
		for each(p2Collider* collider in body->GetColliders())
		{
			//delete(collider);
		}
		delete(body);
	}

	if (m_DebugDraw != nullptr) {
		//delete(m_DebugDraw);
	}
}

void p2World::Step(float dt)
{
	for each (p2Body* body in m_BodyList)
	{
		body->Step(dt);
	}
}

p2Body * p2World::CreateBody(p2BodyDef* bodyDef)
{
	p2Body* tmpBody = new p2Body(*bodyDef, this);
	m_BodyList.push_front(tmpBody);
	auto it = m_BodyList.begin();
	return *it;
}

void p2World::SetContactListener(p2ContactListener * contactListener)
{
}

p2Vec2 p2World::GetGravity()
{
	return m_Gravity;
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

	for (p2Body* body : m_BodyList) {
		//Draw aabb of body
		if (flags & p2Draw::aabbBit) {
			p2Vec2 vertices[4];
			vertices[0] = body->aabb.bottomLeft;
			vertices[1] = p2Vec2(body->aabb.topRight.x, body->aabb.bottomLeft.y);
			vertices[2] = body->aabb.topRight;
			vertices[3] = p2Vec2(body->aabb.bottomLeft.x, body->aabb.topRight.y);
			m_DebugDraw->DrawPolygon(vertices, 4, p2Color(0, 100, 0));
		}

		//Draw center of mass
		if (flags & p2Draw::centerOfMassBit) {
			// TO DO 
			// CHANGE FOR CENTER OF MASS NOT POSITION
			m_DebugDraw->DrawTransform(body->GetPosition());
		}

		for (p2Collider* collider : body->GetColliders()) {
			//Draw collider
			if (flags & p2Draw::colliderBit) {
				if (collider->GetShapeType() == p2ColliderDef::ShapeType::RECT){
					p2RectShape * rect = static_cast<p2RectShape*>(collider->GetShape());

					p2Vec2 size = rect->GetSize();
					p2Vec2 position = collider->GetPosition() - (size / 2);

					m_DebugDraw->DrawRectFilled(position, size, p2Color(0, 0, 100));
				}

				if (collider->GetShapeType() == p2ColliderDef::ShapeType::CIRCLE){
					p2CircleShape *circle = static_cast<p2CircleShape*>(collider->GetShape());

					p2Vec2 center = collider->GetPosition();
					float radius = circle->GetRadius();

					m_DebugDraw->DrawCircleFilled(center, radius, p2Color(0, 0, 100));
					
				}
			}

			//Draw aabb of collider
			if (flags & p2Draw::aabbColliderBit) {
				p2Vec2 vertices[4];
				vertices[0] = collider->aabb.bottomLeft;
				vertices[1] = p2Vec2(collider->aabb.topRight.x, collider->aabb.bottomLeft.y);
				vertices[2] = collider->aabb.topRight;
				vertices[3] = p2Vec2(collider->aabb.bottomLeft.x, collider->aabb.topRight.y);
				m_DebugDraw->DrawPolygon(vertices, 4, p2Color(0, 100, 0));
			}
		}
	}
}

p2Draw * p2World::GetDebugDraw()
{
	return m_DebugDraw;
}
