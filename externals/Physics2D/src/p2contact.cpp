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
	return m_ColliderA->aabb.Overlap(&m_ColliderB->aabb);
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
		p2Manifold manifold;
			
		contact->Update(*m_ContactListener, manifold);

		if (manifold.contact) {
			//Change position to not be intersecting 

			manifold.bodyA->SetPosition(manifold.bodyA->GetPosition() - p2Mat22::RotationMatrix(manifold.bodyA->GetAngle()) * (manifold.normal * (manifold.penetration / 2)));
			manifold.bodyB->SetPosition(manifold.bodyB->GetPosition() + p2Mat22::RotationMatrix(manifold.bodyB->GetAngle()) * (manifold.normal * (manifold.penetration / 2)));

			//Compute Impulse

			//TO REMOVE
			PointsToDraw.push_back(manifold.closetPoint);
			//TO REMOVE

			p2Vec2 relativeVelocity = manifold.bodyB->GetLinearVelocity() - manifold.bodyA->GetLinearVelocity();

			float velocityAlongNormal = p2Vec2::Dot(relativeVelocity, manifold.normal);

			//Do not resolve if velocities is pulling appart both body
			if(velocityAlongNormal > 0){
			break;
			}

			float restitution = std::fmin(contact->GetColliderA()->GetRestitution(), contact->GetColliderB()->GetRestitution());
			
			float impulseScalar = -(1 + restitution) * velocityAlongNormal;
			impulseScalar /= 1 / manifold.bodyA->GetMass() + 1 / manifold.bodyB->GetMass();

			p2Vec2 impulse = manifold.normal * impulseScalar;

			manifold.bodyA->SetLinearVelocity(manifold.bodyA->GetLinearVelocity() - impulse * (1 / manifold.bodyA->GetMass()) );
			manifold.bodyB->SetLinearVelocity(manifold.bodyB->GetLinearVelocity() + impulse * (1 / manifold.bodyB->GetMass()) );
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
	for (p2Contact* contact : m_ContactList) {
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
	}

	//Vector from center to corner
	for (p2Vec2 point : PointsToDraw) {
		debugDraw->DrawCircleFilled(point, 0.05, p2Color(0, 255, 0));
	}

	PointsToDraw.clear();
}

bool SAT::CheckCollisionSAT(p2Contact * contact, p2Manifold& manifold)
{
	switch (contact->GetColliderA()->GetShapeType()) {
	case p2ColliderDef::ShapeType::CIRCLE:
		switch (contact->GetColliderB()->GetShapeType()) {
		case p2ColliderDef::ShapeType::CIRCLE:
			return CheckCollisionCircles(contact, manifold);
			break;

		case p2ColliderDef::ShapeType::RECT:
			return CheckCollisionCircleRect(contact, manifold);
			break;

		case p2ColliderDef::ShapeType::POLYGON:
			return CheckCollisionPolygonCircle(contact, manifold);
			break;
		}
		break;

	case p2ColliderDef::ShapeType::RECT:
		switch (contact->GetColliderB()->GetShapeType()) {
		case p2ColliderDef::ShapeType::CIRCLE:
			return CheckCollisionCircleRect(contact, manifold);
			break;

		case p2ColliderDef::ShapeType::RECT:
			return CheckCollisionRects(contact, manifold);
			break;

		case p2ColliderDef::ShapeType::POLYGON:
			return CheckCollisionPolygonRect(contact, manifold);
			break;
		}
		break;

	case p2ColliderDef::ShapeType::POLYGON:
		switch (contact->GetColliderB()->GetShapeType()) {
		case p2ColliderDef::ShapeType::CIRCLE:
			return CheckCollisionPolygonCircle(contact, manifold);
			break;

		case p2ColliderDef::ShapeType::RECT:
			return CheckCollisionPolygonRect(contact, manifold);
			break;

		case p2ColliderDef::ShapeType::POLYGON:
			return CheckCollisionPolygons(contact, manifold);
			break;
		}
		break;
	}
	return false;
}

bool SAT::CheckCollisionRects(p2Contact * contact, p2Manifold& manifold)
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

	float mtv = std::numeric_limits<float>::max();
	p2Vec2 normalMinimal;

	//Get MinMax projection on each normal
	bool isSeparated = false;

	bool chooseA = false;

	for (int i = 0; i < 4; i++) { //Force to go through all to have the right normal
		p2Vec2 minMaxA = GetMinMaxProj(vectorsA, 4, normalsA[i]);
		p2Vec2 minMaxB = GetMinMaxProj(vectorsB, 4, normalsA[i]);

		float minA = minMaxA.x; float maxA = minMaxA.y;
		float minB = minMaxB.x; float maxB = minMaxB.y;

		isSeparated = maxA < minB || maxB < minA;
		if (isSeparated) {
			break;
		}
		else {
			if (mtv > maxA - minB) {
				mtv = maxA - minB;
				normalMinimal = normalsA[i];
			}
			else if(mtv > maxB - minA){
				mtv = maxB - minA;
				normalMinimal = normalsA[i] * -1;
			}
		}
	}
	if (!isSeparated) {
		for (int i = 0; i < 4; i++) {
			p2Vec2 minMaxA = GetMinMaxProj(vectorsA, 4, normalsB[i]);
			p2Vec2 minMaxB = GetMinMaxProj(vectorsB, 4, normalsB[i]);

			float minA = minMaxA.x; float maxA = minMaxA.y;
			float minB = minMaxB.x; float maxB = minMaxB.y;

			isSeparated = maxA < minB || maxB < minA;
			if (isSeparated) {
				break;
			}
			else {
				if (mtv > maxA - minB) {
					mtv = maxA - minB;
					normalMinimal = normalsB[i];
				}
				else if (mtv > maxB - minA) {
					mtv = maxB - minA;
					normalMinimal = normalsB[i] * -1;
				}
			}
		}
	}

	if (!isSeparated) {
		manifold.penetration = mtv;
		manifold.normal = normalMinimal;
		manifold.contact = true;
	}

	return !isSeparated;
}

bool SAT::CheckCollisionCircles(p2Contact * contact, p2Manifold& manifold)
{
	p2Vec2 distance = (contact->GetColliderB()->GetPosition() - contact->GetColliderA()->GetPosition());

	float radiusTotal = static_cast<p2CircleShape*>(contact->GetColliderA()->GetShape())->GetRadius() + static_cast<p2CircleShape*>(contact->GetColliderB()->GetShape())->GetRadius();

	bool isTouching = distance.GetMagnitude() < radiusTotal;
	
	if (isTouching) {
		manifold.penetration = radiusTotal - distance.GetMagnitude();
		manifold.normal = distance.Normalized();
		manifold.contact = true;

		return true;
	}
	else {
		return false;
	}
}

bool SAT::CheckCollisionCircleRect(p2Contact * contact, p2Manifold& manifold)
{
	//Variable
	p2Collider* colliderA = contact->GetColliderA();
	p2Collider* colliderB = contact->GetColliderB();

	p2RectShape* rect;
	p2Vec2 rectPosition;
	float rectAngle;
	p2Vec2 rectExtends;

	p2CircleShape* circle;
	p2Vec2 circlePosition;

	//Associate variable
	if (colliderA->GetShapeType() == p2ColliderDef::ShapeType::RECT) {
		rect = static_cast<p2RectShape*>(colliderA->GetShape());
		rectPosition = colliderA->GetPosition();
		rectAngle = colliderA->GetBody()->GetAngle();
		rectExtends = rect->GetSize() / 2;

		circle = static_cast<p2CircleShape*>(colliderB->GetShape());
		circlePosition = colliderB->GetPosition();
	}
	else {
		rect = static_cast<p2RectShape*>(colliderB->GetShape());
		rectPosition = colliderB->GetPosition();
		rectAngle = colliderB->GetBody()->GetAngle();
		rectExtends = rect->GetSize() / 2;

		circle = static_cast<p2CircleShape*>(colliderA->GetShape());
		circlePosition = colliderA->GetPosition();
	}

	p2Vec2 rect2circle = circlePosition - rectPosition;

	p2Vec2 unrotedCircle = (p2Mat22::RotationMatrix(rectAngle) * rect2circle) + rectPosition;
		
	p2Vec2 closestPoint;

	//Look for closest point on the rect to the circle

	//Clamp to don't be futher than the rect
	if (unrotedCircle.x > rectPosition.x + rectExtends.x) {
		closestPoint.x = rectPosition.x + rectExtends.x;
	} else if (unrotedCircle.x < rectPosition.x - rectExtends.x) {
		closestPoint.x = rectPosition.x - rectExtends.x;
	} else {
		closestPoint.x = unrotedCircle.x;
	}

	if (unrotedCircle.y > rectPosition.y + rectExtends.y) {
		closestPoint.y = rectPosition.y + rectExtends.y;
	} else if (unrotedCircle.y < rectPosition.y - rectExtends.y) {
		closestPoint.y = rectPosition.y -  rectExtends.y;
	} else {
		closestPoint.y = unrotedCircle.y;
	}

	//If center of circle is inside, clamp to the edge
	if (unrotedCircle == closestPoint) {
		if (abs(unrotedCircle.x - rectPosition.x) > abs(unrotedCircle.y - rectPosition.y)) {
			if (closestPoint.x > rectPosition.x)
				closestPoint.x = rectPosition.x + rectExtends.x;
			else 
				closestPoint.x = rectPosition.x - rectExtends.x;
		} else {
			if (closestPoint.y > rectPosition.y)
				closestPoint.y = rectPosition.y + rectExtends.y;
			else
				closestPoint.y = rectPosition.y - rectExtends.y;
		}
	}

	closestPoint = p2Mat22::RotationMatrix(-rectAngle) * (closestPoint - rectPosition) + rectPosition;

	manifold.closetPoint = closestPoint; //TO REMOVE
	manifold.normal = (manifold.closetPoint - circlePosition).Normalized() * (-1);
	manifold.penetration = circle->GetRadius() - (manifold.closetPoint - circlePosition).GetMagnitude();
	manifold.contact = (manifold.closetPoint - circlePosition).GetMagnitude() < circle->GetRadius();

	return manifold.contact;
}

bool SAT::CheckCollisionPolygons(p2Contact * contact, p2Manifold& manifold)
{
	p2Collider* colliderA = contact->GetColliderA();
	p2Collider* colliderB = contact->GetColliderB();

	p2PolygonShape* polygonA;
	p2Vec2 polygonAPosition;
	float polygonAAngle;

	//Get all vectors and normal from rectB

	p2PolygonShape* polygonB;
	p2Vec2 polygonBPosition;
	float polygonBAngle;

	polygonA = static_cast<p2PolygonShape*>(colliderA->GetShape());
	polygonAPosition = colliderA->GetPosition();
	polygonAAngle = colliderA->GetBody()->GetAngle();

	polygonB = static_cast<p2PolygonShape*>(colliderB->GetShape());
	polygonBPosition = colliderB->GetPosition();
	polygonBAngle = colliderB->GetBody()->GetAngle();

	//Get all vectors and normal from rect
	std::vector<p2Vec2> polygonAVectors;
	std::vector<p2Vec2> polygonANormals;

	polygonAVectors.resize(polygonA->GetVerticesCount());
	polygonANormals.resize(polygonA->GetVerticesCount());
	polygonAVectors = polygonA->GetVerticesWorld(polygonAPosition, polygonAAngle);

	std::vector<p2Vec2> polygonAVectorsVertices;
	polygonAVectorsVertices.resize(polygonA->GetVerticesCount());
	polygonA->GetVectorsVertices(polygonAVectorsVertices, polygonAPosition, polygonAAngle);

	polygonA->GetNormals(polygonANormals, polygonAVectorsVertices);

	//Get all vectors and normal from polygon
	std::vector<p2Vec2> polygonBVectors;
	std::vector<p2Vec2> polygonBNormals;

	polygonBVectors.resize(polygonB->GetVerticesCount());
	polygonBNormals.resize(polygonB->GetVerticesCount());
	polygonBVectors = polygonB->GetVerticesWorld(polygonBPosition, polygonBAngle);

	std::vector<p2Vec2> polygonBVectorsVertices;
	polygonBVectorsVertices.resize(polygonB->GetVerticesCount());
	polygonB->GetVectorsVertices(polygonBVectorsVertices, polygonBPosition, polygonBAngle);

	polygonB->GetNormals(polygonBNormals, polygonBVectorsVertices);

	//Get MinMax projection on each normal
	bool isSeparated = false;

	p2Vec2 normalMinimal;
	float mtv = std::numeric_limits<float>::max();

	for (int i = 0; i < polygonA->GetVerticesCount(); i++) {
		p2Vec2 minMaxA = GetMinMaxProj(polygonAVectors, polygonANormals[i]);
		p2Vec2 minMaxB = GetMinMaxProj(polygonBVectors, polygonANormals[i]);

		float minA = minMaxA.x; float maxA = minMaxA.y;
		float minB = minMaxB.x; float maxB = minMaxB.y;

		isSeparated = maxA < minB || maxB < minA;
		if (isSeparated) {
			break;
		}

		if (mtv > maxA - minB) {
			mtv = maxA - minB;
			normalMinimal = polygonANormals[i];
		}
		else if (mtv > maxB - minA) {
			mtv = maxB - minA;
			normalMinimal = polygonANormals[i] * -1;
		}
	}
	if (!isSeparated) {
		for (int i = 0; i < polygonB->GetVerticesCount(); i++) {
			p2Vec2 minMaxA = GetMinMaxProj(polygonAVectors, polygonBNormals[i]);
			p2Vec2 minMaxB = GetMinMaxProj(polygonBVectors, polygonBNormals[i]);

			float minA = minMaxA.x; float maxA = minMaxA.y;
			float minB = minMaxB.x; float maxB = minMaxB.y;

			isSeparated = maxA < minB || maxB < minA;
			if (isSeparated) {
				break;
			}

			if (mtv > maxA - minB) {
				mtv = maxA - minB;
				normalMinimal = polygonBNormals[i];
			}
			else if (mtv > maxB - minA) {
				mtv = maxB - minA;
				normalMinimal = polygonBNormals[i] * -1;
			}
		}
	}

	if (!isSeparated) {
		manifold.penetration = mtv;
		manifold.normal = normalMinimal;
		manifold.contact = true;
	}

	return !isSeparated;
}

bool SAT::CheckCollisionPolygonRect(p2Contact * contact, p2Manifold& manifold)
{
	p2Collider* colliderA = contact->GetColliderA();
	p2Collider* colliderB = contact->GetColliderB();

	p2RectShape* rect;
	p2Vec2 rectPosition;
	float rectAngle;

	//Get all vectors and normal from rectB

	p2PolygonShape* polygon;
	p2Vec2 polygonPosition;
	float polygonAngle;

	if (colliderA->GetShapeType() == p2ColliderDef::ShapeType::RECT) {
		rect = static_cast<p2RectShape*>(colliderA->GetShape());
		rectPosition = colliderA->GetPosition();
		rectAngle = colliderA->GetBody()->GetAngle();

		polygon = static_cast<p2PolygonShape*>(colliderB->GetShape());
		polygonPosition = colliderB->GetPosition();
		polygonAngle = colliderB->GetBody()->GetAngle();
	}
	else {
		rect = static_cast<p2RectShape*>(colliderB->GetShape());
		rectPosition = colliderB->GetPosition();
		rectAngle = colliderB->GetBody()->GetAngle();

		polygon = static_cast<p2PolygonShape*>(colliderA->GetShape());
		polygonPosition = colliderA->GetPosition();
		polygonAngle = colliderA->GetBody()->GetAngle();
	}

	//Get all vectors and normal from rect
	p2Vec2 rectVectors[4];
	p2Vec2 rectNormals[4];

	rect->GetVectorsCenter(rectVectors, rectPosition, rectAngle);

	p2Vec2 rectVectorsVertices[4];
	rect->GetVectorsVertices(rectVectorsVertices, rectPosition, rectAngle);

	rect->GetNormals(rectNormals, rectVectorsVertices, 4);

	//Get all vectors and normal from polygon
	std::vector<p2Vec2> polygonVectors;
	std::vector<p2Vec2> polygonNormals;

	polygonVectors.resize(polygon->GetVerticesCount());
	polygonNormals.resize(polygon->GetVerticesCount());

	polygonVectors = polygon->GetVerticesWorld(polygonPosition, polygonAngle);

	std::vector<p2Vec2> polygonVectorsVertices;
	polygonVectorsVertices.resize(polygon->GetVerticesCount());
	polygon->GetVectorsVertices(polygonVectorsVertices, polygonPosition, polygonAngle);

	polygon->GetNormals(polygonNormals, polygonVectorsVertices);

	//Get MinMax projection on each normal
	bool isSeparated = false;

	p2Vec2 normalMinimal;
	float mtv = std::numeric_limits<float>::max();

	for (int i = 0; i < 4; i++) {
		p2Vec2 minMaxA = GetMinMaxProj(rectVectors, 4, rectNormals[i]);
		p2Vec2 minMaxB = GetMinMaxProj(polygonVectors, rectNormals[i]);

		float minA = minMaxA.x; float maxA = minMaxA.y;
		float minB = minMaxB.x; float maxB = minMaxB.y;

		isSeparated = maxA < minB || maxB < minA;
		if (isSeparated) {
			break;
		}

		if (mtv > maxA - minB) {
			mtv = maxA - minB;
			normalMinimal = rectNormals[i];
		}
		else if (mtv > maxB - minA) {
			mtv = maxB - minA;
			normalMinimal = rectNormals[i] * -1;
		}
	}
	if (!isSeparated) {
		for (int i = 0; i < polygon->GetVerticesCount(); i++) {
			p2Vec2 minMaxA = GetMinMaxProj(rectVectors, 4, polygonNormals[i]);
			p2Vec2 minMaxB = GetMinMaxProj(polygonVectors, polygonNormals[i]);

			float minA = minMaxA.x; float maxA = minMaxA.y;
			float minB = minMaxB.x; float maxB = minMaxB.y;

			isSeparated = maxA < minB || maxB < minA;

			if (isSeparated) {
				break;
			}

			if (mtv > maxA - minB) {
				mtv = maxA - minB;
				normalMinimal = polygonNormals[i];
			}
			else if (mtv > maxB - minA) {
				mtv = maxB - minA;
				normalMinimal = polygonNormals[i] * -1;
			}
		}
	}

	if (!isSeparated) {
		manifold.penetration = mtv;
		manifold.normal = normalMinimal;
		manifold.contact = true;
	}

	return !isSeparated;
}

bool SAT::CheckCollisionPolygonCircle(p2Contact * contact, p2Manifold& manifold)
{
	//Variables
	p2Collider* colliderA = contact->GetColliderA();
	p2Collider* colliderB = contact->GetColliderB();

	p2PolygonShape* polygon;
	p2Vec2 polygonPosition;
	float polygonAngle;
	p2CircleShape* circle;
	p2Vec2 circlePosition;

	//Associate variables
	if (colliderA->GetShapeType() == p2ColliderDef::ShapeType::POLYGON) {
		polygon = static_cast<p2PolygonShape*>(colliderA->GetShape());
		polygonPosition = colliderA->GetPosition();
		polygonAngle = colliderA->GetBody()->GetAngle();

		circle = static_cast<p2CircleShape*>(colliderB->GetShape());
		circlePosition = colliderB->GetPosition();
	}
	else {
		polygon = static_cast<p2PolygonShape*>(colliderB->GetShape());
		polygonPosition = colliderB->GetPosition();
		polygonAngle = colliderB->GetBody()->GetAngle();

		circle = static_cast<p2CircleShape*>(colliderA->GetShape());
		circlePosition = colliderA->GetPosition();
	}

	//Get closet point of polygon to the circle
	std::vector<p2Vec2> vectorsPolygon;
	vectorsPolygon.resize(polygon->GetVerticesCount());
	vectorsPolygon = polygon->GetVerticesWorld(polygonPosition, polygonAngle);
	
	int indexClosestVertice = 0;
	float minDistance = (circlePosition - vectorsPolygon[0]).GetMagnitude();
	for (int i = 1; i < vectorsPolygon.size(); i++) {
		float curDistance = (circlePosition - vectorsPolygon[i]).GetMagnitude();

		if (curDistance < minDistance) {
			minDistance = curDistance;
			indexClosestVertice = i;
		}
	}

	//Get normals from polygon
	std::vector<p2Vec2> polygonNormals;

	polygonNormals.resize(polygon->GetVerticesCount() + 1);

	std::vector<p2Vec2> polygonVectorsVertices;
	polygonVectorsVertices.resize(polygon->GetVerticesCount());
	polygon->GetVectorsVertices(polygonVectorsVertices, polygonPosition, polygonAngle);

	polygon->GetNormals(polygonNormals, polygonVectorsVertices);

	//Make axis from closet point to center of circle and add this to normals to check
	p2Vec2 polygon2circle = circlePosition - vectorsPolygon[indexClosestVertice];
	p2Vec2 normal = polygon2circle.Normalized();

	polygonNormals.push_back(normal);

	p2Vec2 normalMinimal;
	float mtv = std::numeric_limits<float>::max();

	//Proj
	bool isSeparated = false;
	for (int i = 0; i < polygonNormals.size(); i++) {
		p2Vec2 minMaxA = GetMinMaxProj(vectorsPolygon, polygonNormals[i]);
		float minA = minMaxA.x; float maxA = minMaxA.y;


		float minB = p2Vec2::Dot(circlePosition, polygonNormals[i]) - circle->GetRadius(); 
		float maxB = p2Vec2::Dot(circlePosition, polygonNormals[i]) + circle->GetRadius();

		isSeparated = maxA < minB || maxB < minA;

		if (isSeparated) {
			break;
		}

		if (mtv > maxA - minB) {
			mtv = maxA - minB;
			normalMinimal = polygonNormals[i] * -1;
		}
		else if (mtv > maxB - minA) {
			mtv = maxB - minA;
			normalMinimal = polygonNormals[i];
		}
	}

	if (!isSeparated) {
		manifold.penetration = mtv;
		manifold.normal = normalMinimal;
		manifold.contact = true;
	}

	return !isSeparated;
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

p2Vec2 SAT::GetMinMaxProj(std::vector<p2Vec2>& proj, p2Vec2 axis)
{
	float minProj = p2Vec2::Dot(proj[0], axis);
	float maxProj = p2Vec2::Dot(proj[0], axis);

	for (int i = 1; i < proj.size(); i++) {
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
