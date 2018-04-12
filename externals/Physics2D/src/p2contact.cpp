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
	std::cout << "DESTROY\n";
}

p2Collider * p2Contact::GetColliderA()
{
	return m_ColliderA;
}

p2Collider * p2Contact::GetColliderB()
{
	return m_ColliderB;
}

bool p2Contact::OverlapAABB()
{
	return m_ColliderA->aabb.Overlap(m_ColliderB->aabb);
}

p2ContactManager::p2ContactManager()
{
	m_ContactList = std::list<p2Contact*>();
}

p2ContactManager::~p2ContactManager()
{
}

void p2ContactManager::FindNewContact(std::list<p2Body*> bodies)
{
	//QUAD TREE
	
	// TO CHANGE

	int i = 0;
	for (auto it = bodies.begin(); it != --bodies.end() ; it++) {

		int j = 0;
		for (auto it2 = std::next(it) ; it2 != bodies.end(); it2++) {

			if ((*it)->aabb.Overlap((*it2)->aabb)) {
				for (p2Collider* colliderA : (*it)->GetColliders()) {
					for (p2Collider* colliderB : (*it2)->GetColliders()) {
						if (colliderA->aabb.Overlap(colliderB->aabb)) {
							CreateContact(colliderA, colliderB);
						}
					}
				}
			}

			j++;
		}

		i++;
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
	m_ContactListener->BeginContact(*m_ContactList.begin());
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
	m_ContactListener->EndContact(contact);
	delete(contact);
}
