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
}

p2Body::~p2Body()
{
	for each (p2Collider* collider in m_CollidersList)
	{
		delete(collider);
	}
}

p2Vec2 p2Body::GetLinearVelocity()
{
	return linearVelocity;
}

void p2Body::SetLinearVelocity(p2Vec2 velocity)
{
	linearVelocity = velocity;
}

float p2Body::GetAngularVelocity()
{
	return angularVelocity;
}

p2Vec2 p2Body::GetPosition()
{
	return position;
}

void p2Body::AddForce(p2Vec2 f)
{
	linearVelocity += f;
}

void p2Body::Step(float dt)
{
	if (type == p2BodyType::DYNAMIC) {
		position.x += linearVelocity.x * dt;
		linearVelocity.x += dt * world->GetGravity().x * gravityScale;

		position.y += linearVelocity.y * dt;
		linearVelocity.y += dt * world->GetGravity().y * gravityScale;
	}

}

p2Collider * p2Body::CreateCollider(p2ColliderDef * colliderDef)
{
	p2Collider* tmpCollider = new p2Collider(*colliderDef);
	m_CollidersList.push_front(tmpCollider);
	return tmpCollider;
}

std::list<p2Shape> p2Body::GetShape()
{
	std::list<p2Shape> tmp;

	for each (auto collider in m_CollidersList)
	{
		tmp.push_back(collider->GetShape());
	}

	return tmp;
}
