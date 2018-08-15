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

#if WIN32
#define _CRTDBG_MAP_ALLOC
#include<iostream>
#include <crtdbg.h>
#ifdef _DEBUG
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif
#endif

#include <p2body.h>
#include <p2collider.h>
#include <p2world.h>
#include <p2matrix.h>

#include <iostream>
#include <cmath>

p2Body::p2Body()
{
}

p2Body::p2Body(p2BodyDef bodyDef, p2World* world)
{
	m_Position = bodyDef.position;
	m_LinearVelocity = bodyDef.linearVelocity;
	m_Type = bodyDef.type;
	this->m_World = world;
	m_GravityScale = bodyDef.gravityScale;

	if (m_Type == p2BodyType::STATIC || m_Type == p2BodyType::KINEMATIC) {
		bodyDef.mass = 0;
	}

	m_Mass = bodyDef.mass;
	if (bodyDef.mass == 0) {
		m_InvMass = 0;
	}
	else {
		m_InvMass = 1 / m_Mass;
	}

	m_Angle = bodyDef.angle;
	m_AngularVelocity = 0.0f;
}

p2Body::~p2Body()
{
	if (m_CollidersList.size() > 0) {
		auto it = m_CollidersList.begin();

		while (it != m_CollidersList.end()) {
			delete(*it);
			it = m_CollidersList.erase(it);
		}
	}
}

p2Vec2 p2Body::GetLinearVelocity() const
{
	return m_LinearVelocity;
}

void p2Body::SetLinearVelocity(const p2Vec2 velocity)
{
	m_LinearVelocity = velocity;
}

float p2Body::GetAngularVelocity() const
{
	return m_AngularVelocity;
}

void p2Body::SetAngularVelocity(float angularVelocity)
{
	this->m_AngularVelocity = angularVelocity;
}

p2Vec2 p2Body::GetPosition() const
{
	return m_Position;
}

void p2Body::SetPosition(p2Vec2 position)
{
	ComputeAABB();
	this->m_Position = position;
}

float p2Body::GetAngle() const
{
	return m_Angle;
}

void p2Body::SetAngle(float angle)
{
	m_Angle = angle;
}

void p2Body::AddForce(p2Vec2 f)
{
	m_Force += f;
}

void p2Body::ClearForce()
{
	m_Force = p2Vec2(0, 0);
}

p2Vec2 p2Body::GetForce()
{
	return m_Force;
}

void p2Body::ApplyImpulse(p2Vec2 impulse, p2Vec2 contactPoint)
{
	m_LinearVelocity += impulse * m_InvMass;
	m_AngularVelocity += p2Vec2::Cross(contactPoint, impulse).z * m_InvInertia;
}

p2BodyType p2Body::GetType() const
{
	return m_Type;
}

float p2Body::GetGravityScale() const
{
	return m_GravityScale;
}

void p2Body::SetGravityScale(float gravityScale)
{
	this->m_GravityScale = gravityScale;
}

float p2Body::GetMass() const
{
	return m_Mass;
}

float p2Body::GetInvMass() const
{
	return m_InvMass;
}

p2Collider * p2Body::CreateCollider(p2ColliderDef * colliderDef)
{
	p2Collider* tmpCollider = new p2Collider(*colliderDef, this);
	m_CollidersList.push_front(tmpCollider);

	m_Centroide = p2Vec2(0, 0);
	for (p2Collider* collider : m_CollidersList) {
		m_Centroide += collider->GetCentroide();
	}

	m_Centroide /= m_CollidersList.size(); // TO DO compute all centroide

	m_Inertia = tmpCollider->GetInertia(); //TO DO compute all inertia together
	if (m_Inertia == 0) {
		m_InvInertia = 0.0f;
	}
	else {
		m_InvInertia = 1.0f / m_Inertia;
	}

	ComputeAABB();

	return *m_CollidersList.begin();
}

std::list<p2Collider*>& p2Body::GetColliders()
{
	return m_CollidersList;
}

std::list<p2Shape *>& p2Body::GetShape()
{
	std::list<p2Shape *> tmp;

	for(auto& collider : m_CollidersList)
	{
		tmp.push_back(collider->GetShape());
	}

	return tmp;
}

void p2Body::ComputeAABB()
{
	if (m_CollidersList.size() != 0) {

		auto it = m_CollidersList.begin();

		(*it)->GetShape()->ComputeAABB(&(*it)->aabb, (*it)->GetPosition(), m_Angle);

		m_aabb.bottomLeft = (*it)->aabb.bottomLeft;
		m_aabb.topRight = (*it)->aabb.topRight;

		it++;

		for (it; it != m_CollidersList.end(); it++) {
			//Update aabb of all collider
			(*it)->GetShape()->ComputeAABB(&(*it)->aabb, (*it)->GetPosition(), m_Angle);

			//Check for the biggest aabb
			m_aabb.bottomLeft.x = std::fmin(m_aabb.bottomLeft.x, (*it)->aabb.bottomLeft.x);
			m_aabb.bottomLeft.y = std::fmax(m_aabb.bottomLeft.y, (*it)->aabb.bottomLeft.y);
			
			m_aabb.topRight.x = std::fmax(m_aabb.topRight.x, (*it)->aabb.topRight.x);
			m_aabb.topRight.y = std::fmin(m_aabb.topRight.y, (*it)->aabb.topRight.y);
		}
	}
}

const p2AABB & p2Body::GetAABB() const
{
	return m_aabb;
}

p2World * p2Body::GetWorld()
{
	return m_World;
}

p2Vec2 p2Body::GetCentroide()
{
	int count = 0;
	p2Vec2 tmpCentroide = p2Vec2(0, 0);
	for (p2Collider* collider : m_CollidersList) {
		tmpCentroide += collider->GetCentroide();
		tmpCentroide = tmpCentroide / 2.0f;
	}

	tmpCentroide = p2Mat22::RotationMatrix(-m_Angle) * tmpCentroide;
	tmpCentroide += m_Position;

	return tmpCentroide;
}

float p2Body::GetInvInertia()
{
	return m_InvInertia;
}

void p2Body::ApplyExplosiveForce(float explosiveForce, float explosionRadius, p2Vec2 explosionPosition)
{
	p2AABB aabb;
	aabb.bottomLeft = m_Position - p2Vec2(explosionRadius, -explosionRadius);
	aabb.topRight = m_Position + p2Vec2(explosionRadius, -explosionRadius);

	//Get all bodies that will recive force due to explosion
	std::list<p2Body*> bodiesToApplyForce = m_World->CircleOverlap(aabb);

	//Apply a certain amount of force to all bodies 
	for (p2Body* body : bodiesToApplyForce) {
		float distance = (body->m_Position - m_Position).GetMagnitude();

		float force = explosiveForce * (explosionRadius / distance);

		body->AddForce((body->m_Position - m_Position).Normalized() * force);
	}
}
