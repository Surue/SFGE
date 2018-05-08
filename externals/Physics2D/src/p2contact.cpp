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

#include <p2contact.h>
#include <p2Matrix.h>

#include <iostream>
#include <algorithm>

p2Contact::p2Contact()
{
}

p2Contact::p2Contact(p2Collider * colliderA, p2Collider * colliderB)
{
	m_ColliderA = colliderA;
	m_ColliderB = colliderB;
}

p2Contact::~p2Contact()
{
}

void p2Contact::Update(p2ContactListener& contactListener, p2Manifold& manifold)
{
	bool wasTouching = isTouching;
	//Link both body in manifold
	manifold.bodyA = m_ColliderA->GetBody();
	manifold.bodyB = m_ColliderB->GetBody();

	isTouching = SAT::CheckCollisionSAT(this, manifold); // Check if there is a collision or not

	//If there is a collision check if need to make change to velocity;

	//Tell contactListener if is entering ou exiting an collision
	if (isTouching && !wasTouching) {
		contactListener.BeginContact(this);
	}

	if (!isTouching && wasTouching) {
		contactListener.EndContact(this);
	}
}

p2Collider * p2Contact::GetColliderA() const
{
	return m_ColliderA;
}

p2Collider * p2Contact::GetColliderB() const
{
	return m_ColliderB;
}

bool p2Contact::OverlapAABB() const
{
	return m_ColliderA->aabb.Overlap(m_ColliderB->aabb);
}

bool p2Contact::CheckIfCollision(p2Contact& contact)
{
	if (contact.m_ColliderA->GetBody()->GetType() == p2BodyType::DYNAMIC){
		return true;
	}

	if (contact.m_ColliderA->GetBody()->GetType() == p2BodyType::KINEMATIC) {
		if (contact.m_ColliderA->IsSensor() || (contact.m_ColliderB->IsSensor() && contact.m_ColliderB->GetBody()->GetType() != p2BodyType::STATIC)) {
			return true;
		}
	}

	if (contact.m_ColliderB->GetBody()->GetType() == p2BodyType::DYNAMIC) {
		return true;
	}

	if (contact.m_ColliderB->GetBody()->GetType() == p2BodyType::KINEMATIC) {
		if (contact.m_ColliderB->IsSensor() || (contact.m_ColliderA->IsSensor() && contact.m_ColliderA->GetBody()->GetType() != p2BodyType::STATIC)) {
			return true;
		}
	}

	return false;
}

bool p2Contact::ShouldResolveCollision() const
{
	if (m_ColliderA->GetBody()->GetType() == p2BodyType::DYNAMIC && !m_ColliderA->IsSensor()) {
		if (!m_ColliderB->IsSensor()) {
			return true;
		}
	}

	if (m_ColliderB->GetBody()->GetType() == p2BodyType::DYNAMIC && !m_ColliderB->IsSensor()) {
		if (!m_ColliderA->IsSensor()) {
			return true;
		}
	}

	return false;
}

bool p2Contact::SolvePosition(p2Manifold & manifold)
{
	const float k_slop = 0.001f; // Penetration allowance
	const float percent = 0.2f; // Penetration percentage to correct

	float mA = manifold.bodyA->GetInvMass();
	float mB = manifold.bodyB->GetInvMass();

	float iA = manifold.bodyA->GetInvInertia();
	float iB = manifold.bodyB->GetInvInertia();

	p2Vec2 rA = manifold.contactPoint - manifold.bodyA->GetPosition();
	p2Vec2 rB = manifold.contactPoint - manifold.bodyB->GetPosition();

	float penetration = -manifold.penetration;

	float minPenetration = 0.0f;

	if (penetration > minPenetration) {
		minPenetration = penetration;
	}

	float C = percent * (penetration + k_slop);
	if (C > 0.0f) {
		C = 0.0f;
	}

	if (C < -0.2f) {
		C = -0.2f;
	}

	float rnA = p2Vec2::Cross(rA, manifold.normal).z;
	float rnB = p2Vec2::Cross(rB, manifold.normal).z;
	float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

	float impulse = 0.0f;

	if (K > 0.0f) {
		impulse = -C / K;
	}

	p2Vec2 P = manifold.normal * impulse;

	manifold.bodyA->SetPosition(manifold.bodyA->GetPosition() - P * mA);
	manifold.bodyA->SetAngle(manifold.bodyA->GetAngle() - iA * p2Vec2::Cross(rA, P).z);
	
	manifold.bodyB->SetPosition(manifold.bodyB->GetPosition() + P * mB);
	manifold.bodyB->SetAngle(manifold.bodyB->GetAngle() + iB * p2Vec2::Cross(rB, P).z);
	
	return minPenetration >= -3.0f * k_slop;
}

void p2Contact::SolveVelocity(p2Manifold & manifold)
{
	float mA = manifold.bodyA->GetInvMass();
	float mB = manifold.bodyB->GetInvMass();

	float iA = (manifold.bodyA->GetInvInertia());
	float iB = (manifold.bodyB->GetInvInertia());

	p2Vec2 vA = manifold.bodyA->GetLinearVelocity();
	float wA = manifold.bodyA->GetAngularVelocity();

	p2Vec2 vB = manifold.bodyB->GetLinearVelocity();
	float wB = manifold.bodyB->GetAngularVelocity();

	p2Vec2 tangent = p2Vec2(manifold.normal.y, -manifold.normal.x);

	p2Vec2 rA = manifold.contactPoint - manifold.bodyA->GetCentroide();
	p2Vec2 rB = manifold.contactPoint - manifold.bodyB->GetCentroide();

	float rtA = p2Vec2::Cross(rA, tangent).z;
	float rtB = p2Vec2::Cross(rB, tangent).z;

	p2Vec2 relativeVelocity = vB + p2Vec2(-wB * rB.y, wB * rB.x) - vA - p2Vec2(-wA * rA.y, wA * rA.x);

	float velocityAlongNormal = p2Vec2::Dot(relativeVelocity, manifold.normal);

	//Do not resolve if velocities is pulling appart both body
	if (velocityAlongNormal > 0) {
		return;
	}

	//tangent
	float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

	float tangentMass = 0.0f;

	if (kTangent > 0.0f) {
		tangentMass = 1.0f / kTangent;
	}

	float velocityT = p2Vec2::Dot(relativeVelocity, p2Vec2(manifold.normal.y, -manifold.normal.x));
	float lambda = tangentMass * (-velocityT);
	
	float friction = std::fmin(GetColliderA()->GetFriction(), GetColliderB()->GetFriction());

	float maxFriction = friction * manifold.normalImpulse;
	float newImpulse = manifold.tangentImpulse + lambda;

	if (newImpulse > maxFriction) newImpulse = maxFriction;
	if (newImpulse < -maxFriction) newImpulse = -maxFriction;

	lambda = newImpulse - manifold.tangentImpulse;
	manifold.tangentImpulse = newImpulse;

	p2Vec2 P = tangent * lambda;

	vA -= P * mA;
	wA -= p2Vec2::Cross(rA, P).z * iA;

	vB += P * mB;
	wB += p2Vec2::Cross(rB, P).z * iB;


	//Normal
	float vn = p2Vec2::Dot(relativeVelocity, manifold.normal);

	float kNormal = mA + mB + iA * p2Vec2::Cross(rA, manifold.normal).z * p2Vec2::Cross(rA, manifold.normal).z + iB * p2Vec2::Cross(rB, manifold.normal).z * p2Vec2::Cross(rB, manifold.normal).z;

	float normalMass = 0.0f;
	if (kNormal > 0.0f) {
		normalMass = 1.0f / kNormal;
	}

	float vRel = p2Vec2::Dot(manifold.normal, vB + p2Vec2(-wB * rB.y, wB * rB.x) - vA - p2Vec2(-wA * rA.y, wA * rA.x));
	float restitution = std::fmin(GetColliderA()->GetRestitution(), GetColliderB()->GetRestitution());

	float velocityBias = -restitution * vRel;
	lambda = -normalMass * (vn - velocityBias);

	newImpulse = manifold.normalImpulse + lambda;
	if (newImpulse < 0.0f) newImpulse = 0.0f;
	lambda = newImpulse - manifold.normalImpulse;
	manifold.normalImpulse = newImpulse;

	P = manifold.normal * lambda;
	vA -= P * mA;
	wA -= iA * p2Vec2::Cross(rA, P).z;

	vB += P * mB;
	wB += iB * p2Vec2::Cross(rB, P).z;

	manifold.bodyA->SetLinearVelocity(vA);
	manifold.bodyA->SetAngularVelocity(wA);

	manifold.bodyB->SetLinearVelocity(vB);
	manifold.bodyB->SetAngularVelocity(wB);
}

bool p2Contact::isOnContact()
{
	return isTouching;
}

p2ContactManager::p2ContactManager()
{
	m_ContactList = std::list<p2Contact*>();
	m_QuadTree = p2QuadTree();
}

p2ContactManager::~p2ContactManager()
{
}

void p2ContactManager::FindNewContact(std::list<p2Body*>& bodies)
{
	//QUAD TREE
	m_QuadTree.Clear();

	p2AABB fullAABB;
	auto it = bodies.begin();
	fullAABB.bottomLeft = (*it)->GetAABB().bottomLeft;
	fullAABB.topRight = (*it)->GetAABB().topRight;
	it++;
	
	while (it != bodies.end())
	{
		//Check the biggest aabb
		fullAABB.bottomLeft.x = fmin(fullAABB.bottomLeft.x, (*it)->GetAABB().bottomLeft.x);
		fullAABB.bottomLeft.y = fmax(fullAABB.bottomLeft.y, (*it)->GetAABB().bottomLeft.y);

		fullAABB.topRight.x = fmax(fullAABB.topRight.x, (*it)->GetAABB().topRight.x);
		fullAABB.topRight.y = fmin(fullAABB.topRight.y, (*it)->GetAABB().topRight.y);

		it++;
	}

	fullAABB.bottomLeft += p2Vec2(-0.1, 0.1);
	fullAABB.topRight += p2Vec2(0.1, -0.1);

	m_QuadTree.SetAABB(fullAABB);

	for (p2Body* body : bodies) {
		m_QuadTree.Insert(body);
	}

	std::list<p2Contact> allContact = std::list<p2Contact>();
	m_QuadTree.Retrieve(allContact);

	for(p2Contact contact : allContact) {
		if (p2Contact::CheckIfCollision(contact)) {
			CreateContact(contact.GetColliderA(), contact.GetColliderB());
		}
	}
}

void p2ContactManager::Solve()
{
	for (p2Contact* contact : m_ContactList) {
		p2Manifold manifold;
			
		contact->Update(*m_ContactListener, manifold);

		//Correction collision
		if (manifold.ShouldResolve) {
			//Compute Impulse

			//TO REMOVE
			PointsToDraw.push_back(manifold.contactPoint);
			//TO REMOVE

			for (int i = 0; i < 10; i++) {
				contact->SolveVelocity(manifold);
			}

			for (int i = 0; i < 10; i++) {
				if (contact->SolvePosition(manifold)) {
					break;
				}
			}
		}
	}

}

void p2ContactManager::Collide()
{
	auto it = m_ContactList.begin();

	while (it != m_ContactList.end()) {
		if (!(*it)->OverlapAABB()) {
			Destroy(*it);
			it = m_ContactList.erase(it);
		}
		else {
			it++;
		}
	}
}

void p2ContactManager::SetContactListener(p2ContactListener * contactListener)
{
	m_ContactListener = contactListener;
}

void p2ContactManager::CreateContact(p2Collider * colliderA, p2Collider * colliderB)
{
	for (p2Contact* contact : m_ContactList) {
		if (colliderA == contact->GetColliderA() && colliderB == contact->GetColliderB() || 
			colliderA == contact->GetColliderB() && colliderB == contact->GetColliderA() ||
			colliderA == colliderB) {
			return;
		}
	}
	m_ContactList.push_front(new p2Contact(colliderA, colliderB));
}

void p2ContactManager::Destroy()
{
	auto it = m_ContactList.begin();
	while (it != m_ContactList.end()) {
		delete(*it);

		it = m_ContactList.erase(it);
	}
}

void p2ContactManager::Destroy(p2Contact *contact)
{
	if (contact->isOnContact()) {
		m_ContactListener->EndContact(contact);
	}
	delete(contact);
}

p2QuadTree * p2ContactManager::GetQuadTree()
{
	return &m_QuadTree;
}

void p2ContactManager::Draw(p2Draw* debugDraw)
{
	//Corner
	/*for (p2Contact* contact : m_ContactList) {
		p2Vec2 cornersA[4];
		switch (contact->GetColliderA()->GetShapeType()) {
		case p2ColliderDef::RECT:
			static_cast<p2RectShape*>(contact->GetColliderA()->GetShape())->GetCorners(cornersA, contact->GetColliderA()->GetPosition(), contact->GetColliderA()->GetBody()->GetAngle());

			for (int i = 0; i < 4; i++) {
				debugDraw->DrawCircleFilled(cornersA[i], 0.05, p2Color(255, 0, 0));
			}
			break;

		case p2ColliderDef::POLYGON:
			std::vector<p2Vec2> tmp = static_cast<p2PolygonShape*>(contact->GetColliderA()->GetShape())->GetVerticesWorld(contact->GetColliderA()->GetPosition(), contact->GetColliderA()->GetBody()->GetAngle());

			for (int i = 0; i < tmp.size(); i++) {
				debugDraw->DrawCircleFilled(tmp[i], 0.05, p2Color(255, 0, 0));
			}
			break;
		}
		p2Vec2 cornersB[4];

		switch (contact->GetColliderB()->GetShapeType()) {
		case p2ColliderDef::RECT:
			static_cast<p2RectShape*>(contact->GetColliderB()->GetShape())->GetCorners(cornersB, contact->GetColliderB()->GetPosition(), contact->GetColliderB()->GetBody()->GetAngle());

			for (int i = 0; i < 4; i++) {
				debugDraw->DrawCircleFilled(cornersB[i], 0.05, p2Color(255, 0, 0));
			}
			break;

		case p2ColliderDef::POLYGON:
			std::vector<p2Vec2> tmp = static_cast<p2PolygonShape*>(contact->GetColliderB()->GetShape())->GetVerticesWorld(contact->GetColliderB()->GetPosition(), contact->GetColliderB()->GetBody()->GetAngle());
			
			
			for (int i = 0; i < tmp.size(); i++) {
				debugDraw->DrawCircleFilled(tmp[i], 0.05, p2Color(255, 0, 0));
			}
			break;
		}
	}*/

	//Vector from center to corner
	for (p2Vec2 point : PointsToDraw) {
		debugDraw->DrawCircleFilled(point, 0.05, p2Color(0, 255, 0));
	}

	PointsToDraw.clear();
}
