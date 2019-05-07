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

#include <physics/physics.h>
#include <physics/collider.h>
#include <physics/body2d.h>

namespace sfge
{

const float PhysicsManager::pixelPerMeter = 100.0f;


void PhysicsManager::Init()
{
	p2Vec2 gravity = m_Engine.GetConfig()->gravity;
	m_World = new p2World(gravity);
	m_ContactListener = new ContactListener();
	m_World->SetContactListener(m_ContactListener);

}

void PhysicsManager::Update(sf::Time dt)
{
	if (m_World)
	{
		m_World->Step(dt.asSeconds());
	}
}

p2World * PhysicsManager::GetWorld()
{
	return m_World;
}


void PhysicsManager::Destroy()
{
	if (m_World)
	{
		delete(m_World);
		m_World = nullptr;
	}
	if(m_ContactListener)
	{
		delete(m_ContactListener);
		m_ContactListener = nullptr;
	}

	for (Body2d* body : m_Bodies)
	{
		delete(body);
	}
	m_Bodies.clear();

	for (Collider* collider : m_Colliders)
	{
		delete(collider);
	}
	m_Colliders.clear();
}

void PhysicsManager::Reset()
{
	Destroy();
	Init();
}

void PhysicsManager::Collect()
{
}

Body2d* PhysicsManager::Raycast(sf::Vector2f direction, sf::Vector2f position, float maxDistance)
{
	p2Body* bodyPhysic = m_World->Raycast(sfge::pixel2meter(direction), sfge::pixel2meter(position), sfge::pixel2meter(maxDistance));

	for (Body2d* body : m_Bodies) {
		if (body->GetBody() == bodyPhysic) {
			return body;
		}
	}
}

void PhysicsManager::SetIterativeResolution(int iterativeCount)
{
	m_World->GetContactManager().SetIteration(iterativeCount);
}

void ContactListener::BeginContact(p2Contact* contact) 
{
	Collider* firstCollider = nullptr;
	Collider* secondCollider = nullptr;

	void* colliderUserData = contact->GetColliderA()->GetUserData();
	if (colliderUserData != nullptr) 
	{
		firstCollider = static_cast<Collider*>(colliderUserData);
	}

	
	colliderUserData = contact->GetColliderB()->GetUserData();
	if (colliderUserData != nullptr)
	{
		secondCollider = static_cast<Collider*>(colliderUserData);
	}

	if (firstCollider and secondCollider)
	{
		//Log::GetInstance()->Msg("CONTACT ENTER");
		firstCollider->OnColliderEnter(secondCollider);
		secondCollider->OnColliderEnter(firstCollider);
	}

}

void ContactListener::EndContact(p2Contact* contact)
{

	Collider* firstCollider = nullptr;
	Collider* secondCollider = nullptr;

	void* colliderUserData = contact->GetColliderA()->GetUserData();
	if (colliderUserData != nullptr)
	{
		firstCollider = static_cast<Collider*>(colliderUserData);
	}


	colliderUserData = contact->GetColliderB()->GetUserData();
	if (colliderUserData != nullptr)
	{
		secondCollider = static_cast<Collider*>(colliderUserData);
	}

	if (firstCollider and secondCollider)
	{

		//Log::GetInstance()->Msg("CONTACT EXIT");
		firstCollider->OnColliderExit(secondCollider);
		secondCollider->OnColliderExit(firstCollider);
	}

}


float pixel2meter(float pixel)
{
	return pixel / PhysicsManager::pixelPerMeter;
}

float pixel2meter(int pixel)
{
	return pixel / PhysicsManager::pixelPerMeter;
}

p2Vec2 pixel2meter(sf::Vector2f pixel)
{
	return p2Vec2(pixel2meter(pixel.x), pixel2meter(pixel.y));
}

p2Vec2 pixel2meter(sf::Vector2i pixel)
{
	return p2Vec2(pixel2meter(pixel.x), pixel2meter(pixel.y));
}

float meter2pixel(float meter)
{
	return meter * PhysicsManager::pixelPerMeter;
}

sf::Vector2f meter2pixel(p2Vec2 meter)
{
	return sf::Vector2f(meter2pixel(meter.x), meter2pixel(meter.y));
}

}
