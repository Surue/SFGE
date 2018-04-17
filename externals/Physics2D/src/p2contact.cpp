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
	//Check if is touching via SAT algorithm

	//	Get vectors & normals of collider
	p2Vec2 vectorsA[4];
	p2Vec2 normalsA[4];

	switch (m_ColliderA->GetShapeType()) {
	case p2ColliderDef::ShapeType::CIRCLE:
		std::cout << "C'est un cercle, la fonction n'est pas encore en place\n";
		break;

	case p2ColliderDef::ShapeType::RECT:
		static_cast<p2RectShape*>(m_ColliderA->GetShape())->GetVectorsCenter(vectorsA, m_ColliderA->GetPosition(), m_ColliderA->GetBody()->GetAngle());

		p2Vec2 vectorsVerticesA[4];
		static_cast<p2RectShape*>(m_ColliderA->GetShape())->GetVectorsVertices(vectorsVerticesA, m_ColliderA->GetPosition(), m_ColliderA->GetBody()->GetAngle());
		
		m_ColliderA->GetShape()->GetNormals(normalsA, vectorsVerticesA, 4);
		break;
	}

	p2Vec2 vectorsB[4];
	p2Vec2 normalsB[4];

	switch (m_ColliderB->GetShapeType()) {
	case p2ColliderDef::ShapeType::CIRCLE:
		std::cout << "C'est un cercle, la fonction n'est pas encore en place\n";
		break;

	case p2ColliderDef::ShapeType::RECT:
		static_cast<p2RectShape*>(m_ColliderB->GetShape())->GetVectorsCenter(vectorsB, m_ColliderB->GetPosition(), m_ColliderB->GetBody()->GetAngle());

		p2Vec2 vectorsVerticesB[4];
		static_cast<p2RectShape*>(m_ColliderB->GetShape())->GetVectorsVertices(vectorsVerticesB, m_ColliderB->GetPosition(), m_ColliderB->GetBody()->GetAngle());

		m_ColliderB->GetShape()->GetNormals(normalsB, vectorsVerticesB, 4);
		break;
	}

	//  Get MinMax projection on each normal
	bool isSeparated = false;

	for (int i = 0; i < 2; i++) {
		//For A
		float minProjA = p2Vec2::Dot(vectorsA[0], normalsA[i]);
		float maxProjA = p2Vec2::Dot(vectorsA[0], normalsA[i]);

		for (int j = 1; j < 4; j++) {
			float curProj = p2Vec2::Dot(vectorsA[j], normalsA[i]);
			if (minProjA > curProj) {
				minProjA = curProj;
			}
			if (maxProjA < curProj) {
				maxProjA = curProj;
			}
		}

		//For B
		float minProjB = p2Vec2::Dot(vectorsB[0], normalsA[i]);
		float maxProjB = p2Vec2::Dot(vectorsB[0], normalsA[i]);

		for (int j = 1; j < 4; j++) {
			float curProj = p2Vec2::Dot(vectorsB[j], normalsA[i]);
			if (minProjB > curProj) {
				minProjB = curProj;
			}
			if (maxProjB < curProj) {
				maxProjB = curProj;
			}
		}

		isSeparated = maxProjA < minProjB || maxProjB < minProjA;
		if (isSeparated) {
			break;
		}
	}

	if (!isSeparated) {
		for (int i = 0; i < 2; i++) {
			//For A
			float minProjA = p2Vec2::Dot(vectorsA[0], normalsB[i]);
			float maxProjA = p2Vec2::Dot(vectorsA[0], normalsB[i]);

			for (int j = 1; j < 4; j++) {
				float curProj = p2Vec2::Dot(vectorsA[j], normalsB[i]);
				if (minProjA > curProj) {
					minProjA = curProj;
				}
				if (maxProjA < curProj) {
					maxProjA = curProj;
				}
			}

			//For B
			float minProjB = p2Vec2::Dot(vectorsB[0], normalsB[i]);
			float maxProjB = p2Vec2::Dot(vectorsB[0], normalsB[i]);

			for (int j = 1; j < 4; j++) {
				float curProj = p2Vec2::Dot(vectorsB[j], normalsB[i]);
				if (minProjB > curProj) {
					minProjB = curProj;
				}
				if (maxProjB < curProj) {
					maxProjB = curProj;
				}
			}

			isSeparated = maxProjA < minProjB || maxProjB < minProjA;
			if (isSeparated) {
				break;
			}
		}
	}

	if (!isSeparated) {
		isTouching = true;
	}
	else {
		isTouching = false;
	}

	//	Check if is separated

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
