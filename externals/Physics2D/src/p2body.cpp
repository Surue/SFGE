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

#include <p2body.h>
#include <p2Collider.h>
#include <p2World.h>

#include <iostream>

p2Body::p2Body()
{
}

p2Body::p2Body(p2BodyDef bodyDef, p2World* world)
{
	position = bodyDef.position;
	linearVelocity = bodyDef.linearVelocity;
	type = bodyDef.type;
	this->world = world;
	gravityScale = bodyDef.gravityScale;

	m_Mass = bodyDef.mass;

	m_Angle = 0.0f;
}

p2Body::~p2Body()
{
	auto it = m_CollidersList.begin();

	while (it != m_CollidersList.end()) {
		delete(*it);
		it = m_CollidersList.erase(it);
	}
}

p2Vec2 p2Body::GetLinearVelocity() const
{
	return linearVelocity;
}

void p2Body::SetLinearVelocity(p2Vec2 velocity)
{
	linearVelocity = velocity;
}

float p2Body::GetAngularVelocity() const
{
	return angularVelocity;
}

p2Vec2 p2Body::GetPosition() const
{
	return position;
}

void p2Body::SetPosition(p2Vec2 position)
{
	ComputeAABB();
	this->position = position;
}

float p2Body::GetAngle() const
{
	return m_Angle;
}

void p2Body::SetAngle(float angle)
{
	ComputeAABB();
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

p2BodyType p2Body::GetType() const
{
	return type;
}

float p2Body::GetGravityScale() const
{
	return gravityScale;
}

float p2Body::GetMass() const
{
	return m_Mass;
}

p2Collider * p2Body::CreateCollider(p2ColliderDef * colliderDef)
{
	p2Collider* tmpCollider = new p2Collider(*colliderDef, this);
	m_CollidersList.push_front(tmpCollider);

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

	for each (auto collider in m_CollidersList)
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

		aabb.bottomLeft = (*it)->aabb.bottomLeft;
		aabb.topRight = (*it)->aabb.topRight;

		it++;

		for (it; it != m_CollidersList.end(); it++) {
			//Update aabb of all collider
			(*it)->GetShape()->ComputeAABB(&(*it)->aabb, (*it)->GetPosition(), m_Angle);

			//Check for the biggest aabb
			aabb.bottomLeft.x = fmin(aabb.bottomLeft.x, (*it)->aabb.bottomLeft.x);
			aabb.bottomLeft.y = fmax(aabb.bottomLeft.y, (*it)->aabb.bottomLeft.y);
			
			aabb.topRight.x = fmax(aabb.topRight.x, (*it)->aabb.topRight.x);
			aabb.topRight.y = fmin(aabb.topRight.y, (*it)->aabb.topRight.y);
		}
	}
}

const p2AABB * p2Body::GetAABB() const
{
	return &aabb;
}
