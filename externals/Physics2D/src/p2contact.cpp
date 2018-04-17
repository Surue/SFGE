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

#include <iostream>

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

void p2Contact::Update(p2ContactListener& contactListener)
{
	bool wasTouching = isTouching;
	isTouching = SAT::CheckCollisionSAT(this);

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
	return m_ColliderA->aabb.Overlap(&m_ColliderB->aabb);
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
	fullAABB.bottomLeft = (*it)->GetAABB()->bottomLeft;
	fullAABB.topRight = (*it)->GetAABB()->topRight;
	it++;
	
	while (it != bodies.end())
	{
		//Check the biggest aabb
		fullAABB.bottomLeft.x = fmin(fullAABB.bottomLeft.x, (*it)->GetAABB()->bottomLeft.x);
		fullAABB.bottomLeft.y = fmax(fullAABB.bottomLeft.y, (*it)->GetAABB()->bottomLeft.y);

		fullAABB.topRight.x = fmax(fullAABB.topRight.x, (*it)->GetAABB()->topRight.x);
		fullAABB.topRight.y = fmin(fullAABB.topRight.y, (*it)->GetAABB()->topRight.y);

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
		CreateContact(contact.GetColliderA(), contact.GetColliderB());
	}
}

void p2ContactManager::Solve()
{
	for (p2Contact* contact : m_ContactList) {
		contact->Update(*m_ContactListener);
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
		if (colliderA == contact->GetColliderA() && colliderB == contact->GetColliderB()) {
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
		static_cast<p2RectShape*>(contact->GetColliderA()->GetShape())->GetCorners(cornersA, contact->GetColliderA()->GetPosition(), contact->GetColliderA()->GetBody()->GetAngle());

		for (int i = 0; i < 4; i++) {
			debugDraw->DrawCircleFilled(cornersA[i], 0.05, p2Color(255, 0, 0));
		}

		p2Vec2 cornersB[4];
		static_cast<p2RectShape*>(contact->GetColliderB()->GetShape())->GetCorners(cornersB, contact->GetColliderB()->GetPosition(), contact->GetColliderB()->GetBody()->GetAngle());

		for (int i = 0; i < 4; i++) {
			debugDraw->DrawCircleFilled(cornersB[i], 0.05, p2Color(255, 0, 0));
		}
	}*/

	//Vector from center to corner
}

bool SAT::CheckCollisionSAT(p2Contact * contact)
{
	switch (contact->GetColliderA()->GetShapeType()) {
	case p2ColliderDef::ShapeType::CIRCLE:
		switch (contact->GetColliderB()->GetShapeType()) {
		case p2ColliderDef::ShapeType::CIRCLE:
			return CheckCollisionCircles(contact);
			break;

		case p2ColliderDef::ShapeType::RECT:
			return CheckCollisionCircleRect(contact);
			break;

		case p2ColliderDef::ShapeType::POLYGON:
			return CheckCollisionPolygonCircle(contact);
			break;
		}
		break;

	case p2ColliderDef::ShapeType::RECT:
		switch (contact->GetColliderB()->GetShapeType()) {
		case p2ColliderDef::ShapeType::CIRCLE:
			return CheckCollisionCircleRect(contact);
			break;

		case p2ColliderDef::ShapeType::RECT:
			return CheckCollisionRects(contact);
			break;

		case p2ColliderDef::ShapeType::POLYGON:
			return CheckCollisionPolygonRect(contact);
			break;
		}
		break;

	case p2ColliderDef::ShapeType::POLYGON:
		switch (contact->GetColliderB()->GetShapeType()) {
		case p2ColliderDef::ShapeType::CIRCLE:
			return CheckCollisionPolygonCircle(contact);
			break;

		case p2ColliderDef::ShapeType::RECT:
			return CheckCollisionPolygonRect(contact);
			break;

		case p2ColliderDef::ShapeType::POLYGON:
			return CheckCollisionPolygons(contact);
			break;
		}
		break;
	}
	return false;
}

bool SAT::CheckCollisionRects(p2Contact * contact)
{
	//Get all vectors and normal from rectA
	p2Vec2 vectorsA[4];
	p2Vec2 normalsA[4];

	p2Collider* colliderA = contact->GetColliderA();
	p2RectShape* shapeA = static_cast<p2RectShape*>(colliderA->GetShape());
	p2Vec2 positionA = colliderA->GetPosition();
	float angleA = colliderA->GetBody()->GetAngle();

	shapeA->GetVectorsCenter(vectorsA, positionA, angleA);

	p2Vec2 vectorsVerticesA[4];
	shapeA->GetVectorsVertices(vectorsVerticesA, positionA, angleA);

	shapeA->GetNormals(normalsA, vectorsVerticesA, 4);

	//Get all vectors and normal from rectB
	p2Vec2 vectorsB[4];
	p2Vec2 normalsB[4];

	p2Collider* colliderB = contact->GetColliderB();
	p2RectShape* shapeB = static_cast<p2RectShape*>(colliderB->GetShape());
	p2Vec2 positionB = colliderB->GetPosition();
	float angleB = colliderB->GetBody()->GetAngle();

	shapeB->GetVectorsCenter(vectorsB, positionB, angleB);

	p2Vec2 vectorsVerticesB[4];
	shapeB->GetVectorsVertices(vectorsVerticesB, positionB, angleB);

	shapeB->GetNormals(normalsB, vectorsVerticesB, 4);

	//Get MinMax projection on each normal
	bool isSeparated = false;

	for (int i = 0; i < 2; i++) {
		p2Vec2 minMaxA = GetMinMaxProj(vectorsA, 4, normalsA[i]);
		p2Vec2 minMaxB = GetMinMaxProj(vectorsB, 4, normalsA[i]);

		float minA = minMaxA.x; float maxA = minMaxA.y;
		float minB = minMaxB.x; float maxB = minMaxB.y;

		isSeparated = maxA < minB || maxB < minA;
		if (isSeparated) {
			break;
		}
	}
	if (!isSeparated) {
		for (int i = 0; i < 2; i++) {
			p2Vec2 minMaxA = GetMinMaxProj(vectorsA, 4, normalsB[i]);
			p2Vec2 minMaxB = GetMinMaxProj(vectorsB, 4, normalsB[i]);

			float minA = minMaxA.x; float maxA = minMaxA.y;
			float minB = minMaxB.x; float maxB = minMaxB.y;

			isSeparated = maxA < minB || maxB < minA;
			if (isSeparated) {
				break;
			}
		}
	}

	return !isSeparated;
}

bool SAT::CheckCollisionCircles(p2Contact * contact)
{
	return false;
}

bool SAT::CheckCollisionCircleRect(p2Contact * contact)
{
	return false;
}

bool SAT::CheckCollisionPolygons(p2Contact * contact)
{
	return false;
}

bool SAT::CheckCollisionPolygonRect(p2Contact * contact)
{
	return false;
}

bool SAT::CheckCollisionPolygonCircle(p2Contact * contact)
{
	return false;
}

p2Vec2 SAT::GetMinMaxProj(p2Vec2 proj[], int sizeArray, p2Vec2 axis)
{
	float minProj = p2Vec2::Dot(proj[0], axis);
	float maxProj = p2Vec2::Dot(proj[0], axis);

	for (int i = 1; i < sizeArray; i++) {
		float curProj = p2Vec2::Dot(proj[i], axis);
		if (minProj > curProj) {
			minProj = curProj;
		}
		if (maxProj < curProj) {
			maxProj = curProj;
		}
	}

	return p2Vec2(minProj, maxProj);
}
