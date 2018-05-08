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
#include <p2contactSolver.h>

class p2ContactListener;

/**
* \brief Representation of a contact given as argument in a p2ContactListener
*/
class p2Contact
{
public:
	p2Contact();
	p2Contact(p2Collider *colliderA, p2Collider *colliderB);

	~p2Contact();

	/**
	* \brief called every step, create a manifold containing all information about the collision
	*/
	void Update(p2ContactListener& contactListener, p2Manifold& manifold);

	//Get colliders 
	p2Collider* GetColliderA() const;
	p2Collider* GetColliderB() const;

	/**
	* \brief simple test to remove some contact
	*/
	bool OverlapAABB() const;
	/**
	* \brief return if there is a collision depending on both body type
	*/
	static bool CheckIfCollision(p2Contact& contact);
	/**
	* \brief return true if the collision should be solved
	*/
	bool ShouldResolveCollision() const;
	/**
	* \brief solve position
	*/
	bool SolvePosition(p2Manifold& manifold);
	/**
	* \brief solve velocity
	*/
	void SolveVelocity(p2Manifold& manifold);
	/**
	* \briefe return true if there both collider are touching
	*/
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

	/**
	* \brief use the quadtree to retrive all possible contact
	*/
	void FindNewContact(std::list<p2Body*>& bodies);
	/**
	* \brief loop through all contact to check if there is a collision and resolve it if needed
	*/
	void Solve();
	/**
	* \brief used to remove non colliding contact
	*/
	void Collide();
	/**
	* \brief set the contact listener
	*/
	void SetContactListener(p2ContactListener* contactListener);
	/**
	* \brief factory to create contact
	*/
	void CreateContact(p2Collider* colliderA, p2Collider* colliderB);
	/**
	* \brief clear all contact
	*/
	void Clear();
	/**
	* \brief destroy an unique contact
	*/
	void Destroy(p2Contact *contact);
	/**
	* \brief return the quadtree
	*/
	p2QuadTree* GetQuadTree();
	/**
	* \used to draw debug info
	*/
	void Draw(p2Draw* debugDraw);

	/**
	* \brief list of contact points
	*/
	std::list<p2Vec2> PointsToDraw;

	/**
	* \brief set number of iteration for solving position and velocities
	*/
	void SetIteration(int iterationCount);

private:
	std::list<p2Contact*> m_ContactList;
	p2ContactListener* m_ContactListener;

	p2QuadTree m_QuadTree;

	int m_VelocityCorrectionIteration = 10;
	int m_PositionCorrectionIteration = 10;
};
#endif