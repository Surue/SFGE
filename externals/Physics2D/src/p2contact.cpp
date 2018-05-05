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
	const float k_slop = 0.0005f; // Penetration allowance
	const float percent = 0.8f; // Penetration percentage to correct

	float mA = manifold.bodyA->GetInvMass();
	float mB = manifold.bodyB->GetInvMass();

	float iA = manifold.bodyA->GetInvInertia();
	float iB = manifold.bodyB->GetInvInertia();

	p2Vec2 rA = manifold.contactPoint - manifold.bodyA->GetPosition();
	p2Vec2 rB = manifold.contactPoint - manifold.bodyB->GetPosition();

	float penetration = manifold.penetration;

	float minPenetration = 0.0f;

	if (penetration > minPenetration) {
		minPenetration = penetration;
	}

	float C = percent * (-penetration + k_slop);
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

	//Change position to not be in collision anymore and velocity only if body is dynamic

	if (manifold.bodyA->GetType() == p2BodyType::DYNAMIC) {
		manifold.bodyA->SetPosition(manifold.bodyA->GetPosition() - P * mA);
		manifold.bodyA->SetAngle(manifold.bodyA->GetAngle() - iA * p2Vec2::Cross(rA, P).z);
	}

	if (manifold.bodyB->GetType() == p2BodyType::DYNAMIC) {
		manifold.bodyB->SetPosition(manifold.bodyB->GetPosition() + P * mB);
		manifold.bodyB->SetAngle(manifold.bodyB->GetAngle() + iB * p2Vec2::Cross(rB, P).z);
	}

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
	
	float friction = 0;

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

	if (manifold.bodyA->GetType() == p2BodyType::DYNAMIC) {
		manifold.bodyA->SetLinearVelocity(vA);
		manifold.bodyA->SetAngularVelocity(wA);
	}

	if (manifold.bodyB->GetType() == p2BodyType::DYNAMIC) {
		manifold.bodyB->SetLinearVelocity(vB);
		manifold.bodyB->SetAngularVelocity(wB);
	}
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
		manifold.ShouldResolve = contact->ShouldResolveCollision();

		if (manifold.ShouldResolve) {
			manifold.contactPoint = FindContactPoint(contact, manifold);
		}
	}

	return !isSeparated;
}

bool SAT::CheckCollisionCircles(p2Contact * contact, p2Manifold& manifold)
{
	p2Vec2 distance = (contact->GetColliderB()->GetPosition() - contact->GetColliderA()->GetPosition());

	float radiusA = static_cast<p2CircleShape*>(contact->GetColliderA()->GetShape())->GetRadius();
	float radiusB = static_cast<p2CircleShape*>(contact->GetColliderB()->GetShape())->GetRadius();

	p2Vec2 positionA = contact->GetColliderA()->GetPosition();
	p2Vec2 positionB = contact->GetColliderB()->GetPosition();

	float radiusTotal = radiusA + radiusB;
	
	if (distance.GetMagnitude() < radiusTotal) {
		manifold.normal = distance.Normalized();
		manifold.penetration = p2Vec2::Dot((positionA + manifold.normal * radiusA) - (positionB - manifold.normal * radiusB), manifold.normal);
		manifold.ShouldResolve = contact->ShouldResolveCollision();
		manifold.contactPoint = ((positionA + manifold.normal * radiusA) + (positionB - manifold.normal * radiusB)) * 0.5f;
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

	bool flip = false;

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

		flip = true;
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

	bool isInside = false;

	//If center of circle is inside, clamp to the edge
	if (unrotedCircle == closestPoint) {
		isInside = true;

		float extX = 0;
		if (closestPoint.x < rectPosition.x) {
			extX = rectExtends.x;
		}
		else {
			extX = -rectExtends.x;
		}

		float extY = 0;
		if (closestPoint.y < rectPosition.y) {
			extY = rectExtends.y;
		}
		else {
			extY = -rectExtends.y;
		}

		if (abs(closestPoint.x - rectPosition.x + extX) < abs(closestPoint.y - rectPosition.y + extY)) {
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

	manifold.contactPoint = closestPoint;
	if (isInside || flip) {
		manifold.normal = (circlePosition - manifold.contactPoint).Normalized() * (-1);
	}
	else {
		manifold.normal = (circlePosition - manifold.contactPoint).Normalized();
	}
	manifold.penetration = circle->GetRadius() - (manifold.contactPoint - circlePosition).GetMagnitude();
	manifold.ShouldResolve = ((circlePosition - manifold.contactPoint).GetMagnitude() < circle->GetRadius() || isInside)  && contact->ShouldResolveCollision();

	return (circlePosition - manifold.contactPoint).GetMagnitude() < circle->GetRadius() || isInside;
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
		manifold.ShouldResolve = contact->ShouldResolveCollision();

		if (manifold.ShouldResolve) {
			manifold.contactPoint = FindContactPoint(contact, manifold);
		}
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

	//Must flip if doing from polygon to rect
	bool flip = false; 

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

		flip = true;
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

		manifold.ShouldResolve = contact->ShouldResolveCollision();

		if (flip) {
			manifold.normal = manifold.normal * -1;
		}

		if (manifold.ShouldResolve) {
			manifold.contactPoint = FindContactPoint(contact, manifold);
		}
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
		manifold.ShouldResolve = contact->ShouldResolveCollision();
		manifold.contactPoint = circlePosition + manifold.normal * circle->GetRadius();
	}

	return !isSeparated;
}

p2Vec2 SAT::FindContactPoint(const p2Contact* contact, const p2Manifold & manifold)
{
	p2Collider* colliderA = contact->GetColliderA();
	p2Collider* colliderB = contact->GetColliderB();

	std::vector<p2Vec2> cornersA;
	std::vector<p2Vec2> cornersB;

	bool bothRect = false; //If both are rect, no need to inverse normal later in the code because the normal is already in the right direction
	bool bothPoly = false;
	
	bool rectToPoly = false;

	//We now this function is only called if we have rect or polygon
	if (colliderA->GetShapeType() == p2ColliderDef::ShapeType::POLYGON) {
		p2PolygonShape* shapeA = static_cast<p2PolygonShape*>(colliderA->GetShape());
		cornersA.resize(shapeA->GetVerticesCount());
		cornersA = shapeA->GetVerticesWorld(colliderA->GetPosition(), colliderA->GetBody()->GetAngle());
		bothPoly = true;
		rectToPoly = true;
	}
	else if (colliderA->GetShapeType() == p2ColliderDef::ShapeType::RECT) {
		cornersA.resize(4);
		static_cast<p2RectShape*>(colliderA->GetShape())->GetCorners(cornersA, colliderA->GetPosition(), colliderA->GetBody()->GetAngle());
		bothRect = true;
	}
	else {
		std::cout << "ERROR: FIND CONTACT POINT COLLIDER A\n";
		return manifold.contactPoint;
	}

	if (colliderB->GetShapeType() == p2ColliderDef::ShapeType::POLYGON) {
		p2PolygonShape* shapeB = static_cast<p2PolygonShape*>(colliderB->GetShape());
		cornersB.resize(shapeB->GetVerticesCount());
		cornersB = shapeB->GetVerticesWorld(colliderB->GetPosition(), colliderB->GetBody()->GetAngle());
		bothRect = false;
	}
	else if (colliderB->GetShapeType() == p2ColliderDef::ShapeType::RECT) {
		cornersB.resize(4);
		static_cast<p2RectShape*>(colliderB->GetShape())->GetCorners(cornersB, colliderB->GetPosition(), colliderB->GetBody()->GetAngle());
		bothPoly = false;
		rectToPoly = rectToPoly && true && !bothPoly && !bothRect;
	}
	else {
		std::cout << "ERROR: FIND CONTACT POINT COLLIDER B\n";
		return manifold.contactPoint;
	}

	p2Vec2 normalContact = manifold.normal;

	p2Edge closestA = FindClosestEdge(cornersA, normalContact);
	p2Edge closestB = FindClosestEdge(cornersB, normalContact * (-1));
	
	//std::cout << "( max ) | ( v1 ) | ( v2) \n";
	//std::cout << "-------------------------\n";
	//std::cout << "( " << closestA.max.x << ", " << closestA.max.y << ") | (" << closestA.pointA.x << ", " << closestA.pointA.y << ") | (" << closestA.pointB.x << ", " << closestA.pointB.y << ")\n";
	//std::cout << "( " << closestB.max.x << ", " << closestB.max.y << ") | (" << closestB.pointA.x << ", " << closestB.pointA.y << ") | (" << closestB.pointB.x << ", " << closestB.pointB.y << ")\n";

	//Find the reference and incident edge
	p2Edge ref;
	p2Edge inc;

	bool flip = false;

	if (abs(p2Vec2::Dot(closestA.vector, normalContact)) <= abs(p2Vec2::Dot(closestB.vector, normalContact))) {
		ref = closestA;
		inc = closestB;
	}
	else {
		ref = closestB;
		inc = closestA;
		//std::cout << "flip";
		flip = true;
	}
	
	//Start clipping the points
	p2Vec2 refVector = ref.vector.Normalized();

	//Create the max points
	float offset1 = p2Vec2::Dot(refVector, ref.pointA);;
	float offset2 = p2Vec2::Dot(refVector, ref.pointB);

	//Clip other point with the limit
	std::vector<p2Vec2> clippedPoints;
	clippedPoints.resize(2);

	clippedPoints = ClipPoints(inc.pointA, inc.pointB, refVector, offset1);

	if (clippedPoints.size() < 2) {
		std::cout << "MOTHERFUCKER 1\n";
		return p2Vec2();
	}

	//Same clipping to the other direction
	clippedPoints = ClipPoints(clippedPoints[0], clippedPoints[1], refVector * (-1), -offset2);
	if (clippedPoints.size() < 2) {
		std::cout << "MOTHERFUCKER 2\n";
		return p2Vec2();
	}

	p2Vec2 refNormal = ref.vector.Normal().Normalized();
	
	//Must be upgraded: peut être regarder dans findClosestEdge qui devrait pouvoir savoir s'il doit inverser ou pas la valeur
	if (rectToPoly) refNormal = refNormal * -1;

	if (flip && !bothRect) refNormal = refNormal * -1;

	if (!flip && bothPoly)  refNormal = refNormal * -1;
	//Must be upgraded

	float max = p2Vec2::Dot(refNormal, ref.max);

	if (p2Vec2::Dot(refNormal, clippedPoints[0]) - max >= 0.0f) {
		if (p2Vec2::Dot(refNormal, clippedPoints[1]) - max >= 0.0f) {
			return (clippedPoints[0] + clippedPoints[1]) / 2;
		}
		else {
			return clippedPoints[0];
		}
	}
	return clippedPoints[1];
}

p2Edge SAT::FindClosestEdge(std::vector<p2Vec2> const vertices, p2Vec2 const normal)
{
	int index = 0;
	float maxProj = p2Vec2::Dot(vertices[index], normal);

	//Find the farthest vertex from the normal
	for (int i = 1; i < vertices.size(); i++) {
		float curProj = p2Vec2::Dot(vertices[i], normal);

		if (curProj > maxProj) {
			maxProj = curProj;
			index = i;
		}
	}
	
	//Check the left and right edge from the previous one to see wich one if the most perpendicular to the normal
	p2Vec2 v = vertices[index];
	p2Vec2 v1;
	if (index == vertices.size() - 1) {
		v1 = vertices[0];
	}
	else {
		v1 = vertices[index + 1];
	}
	p2Vec2 v2;
	if (index == 0) {
		v2 = vertices[vertices.size() - 1];
	}
	else {
		v2 = vertices[index - 1];
	}

	p2Vec2 left = (v - v1).Normalized();
	p2Vec2 right = (v - v2).Normalized();

	p2Edge closestEdge;

	if (p2Vec2::Dot(right, normal) <= p2Vec2::Dot(left, normal)) {
		closestEdge.max = v2;
		closestEdge.pointA = v2;
		closestEdge.pointB = v;
		closestEdge.vector = v - v2;
	}
	else {
		closestEdge.max = v;
		closestEdge.pointA = v;
		closestEdge.pointB = v1;
		closestEdge.vector = v1 - v;
	}

	return closestEdge;
}

std::vector<p2Vec2> SAT::ClipPoints(p2Vec2 pointsA, p2Vec2 pointsB, p2Vec2 normal, float proj)
{
	std::vector<p2Vec2> clippedPoints;
	//Projection of points allong the normal
	float d1 = p2Vec2::Dot(pointsA, normal) - proj;
	float d2 = p2Vec2::Dot(pointsB, normal) - proj;
	
	//If the points are past the proj along n
	if (d1 >= 0.0f) clippedPoints.push_back(pointsA);
	if (d2 >= 0.0f) clippedPoints.push_back(pointsB);

	//If they are on opposing side, compute correct point to clip the point to the correct position
	if (d1 * d2 < 0.0) {
		p2Vec2 correctedPoint = pointsB - pointsA;

		float u = d1 / (d1 - d2);

		correctedPoint *= u;

		correctedPoint += pointsA;

		clippedPoints.push_back(correctedPoint);
	}

	return clippedPoints;
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
