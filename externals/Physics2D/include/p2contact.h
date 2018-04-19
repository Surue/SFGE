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

#ifndef SFGE_P2CONTACT_H
#define SFGE_P2CONTACT_H

#include <p2collider.h>
#include <p2body.h>
#include <p2quadtree.h>


class p2ContactListener;

class SAT{
public:
	static bool CheckCollisionSAT(p2Contact* contact);

	static bool CheckCollisionRects(p2Contact* contact);

	static bool CheckCollisionCircles(p2Contact* contact);

	static bool CheckCollisionCircleRect(p2Contact* contact);

	static bool CheckCollisionPolygons(p2Contact* contact);

	static bool CheckCollisionPolygonRect(p2Contact* contact);

	static bool CheckCollisionPolygonCircle(p2Contact* contact);
private:
	SAT();

	static p2Vec2 GetMinMaxProj(p2Vec2 proj[], int sizeArray, p2Vec2 axis);

	static p2Vec2 GetMinMaxProj(std::vector<p2Vec2> proj, p2Vec2 axis);
};
/**
* \brief Representation of a contact given as argument in a p2ContactListener
*/
class p2Contact
{
public:
	p2Contact();
	p2Contact(p2Collider *colliderA, p2Collider *colliderB);

	~p2Contact();

	void Update(p2ContactListener& contactListener);

	p2Collider* GetColliderA() const;
	p2Collider* GetColliderB() const;

	bool OverlapAABB() const;

	bool isOnContact();

private:
	p2Collider* m_ColliderA;
	p2Collider* m_ColliderB;

	bool isTouching = false;
};

/**
* \brief Listener of contacts happening in an attached p2World
*/
class p2ContactListener
{
public:
	virtual void BeginContact(p2Contact* contact) = 0;
	virtual void EndContact(p2Contact* contact) = 0;
};

/**
* \brief Managing the creation and destruction of contact between colliders
*/
class p2ContactManager
{
public:
	p2ContactManager();
	~p2ContactManager();

	void FindNewContact(std::list<p2Body*>& bodies);

	void Solve();

	void Collide();

	void SetContactListener(p2ContactListener* contactListener);

	void CreateContact(p2Collider* colliderA, p2Collider* colliderB);

	void Destroy();
	void Destroy(p2Contact *contact);

	p2QuadTree* GetQuadTree();

	void Draw(p2Draw* debugDraw);

private:
	std::list<p2Contact*> m_ContactList;
	p2ContactListener* m_ContactListener;

	p2QuadTree m_QuadTree;
};
#endif