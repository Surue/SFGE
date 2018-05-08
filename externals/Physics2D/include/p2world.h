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

#ifndef SFGE_P2WORLD_H
#define SFGE_P2WORLD_H

#include <p2vector.h>
#include <p2contact.h>
#include <p2draw.h>

#include <list>
#include <map>

class p2Body;

/**
* \brief Representation of the physical world in meter
*/
class p2World
{
public:
	p2World(p2Vec2 gravity);
	~p2World();

	/**
	* \brief Simulate a new step of the physical world, simplify the resolution with a QuadTree, generate the new contacts
	*/
	void Step(float dt);
	/**
	* \brief Factory method to create a new p2Body attached to the p2World
	*/
	p2Body* CreateBody(p2BodyDef* bodyDef);
	/**
	* \brief Set the contact listener
	*/
	void SetContactListener(p2ContactListener* contactListener);

	//Overlap methods
	/**
	* \brief test if bodies are inside a rect
	* \param aabb representing a box around the circle
	* \return liste of bodies on heap
	*/
	std::list<p2Body*> AABBOverlap(p2AABB aabb);
	/**
	* \brief test if bodies are inside a circle
	* \param aabb representing a box around the circle
	* \return liste of bodies on heap
	*/
	std::list<p2Body*> CircleOverlap(p2AABB aabb);

	//Raycast methods
	/**
	* \brief raycast from a position to a maxDistance and return all object
	* \param vector used as a direction
	* \param position is starting position of object
	* \param maxDistance indicate length of raycast
	* \return a list of bodies touched by the raycast
	*/
	std::list<p2Body*> RaycastAll(p2Vec2 vector, p2Vec2 position, float maxDistance = std::numeric_limits<float>::infinity());
	/**
	* \brief raycast from a position to a maxDistance and return first object touch
	* \param vector used as a direction
	* \param position is starting position of object
	* \param maxDistance indicate length of raycast
	* \return first body touch
	*/
	p2Body* Raycast(p2Vec2 vector, p2Vec2 position, float maxDistance = std::numeric_limits<float>::infinity());
	
	//Debug draw methods
	/**
	* \brief Call for drawing all physical debug data (collider's shape, aabb)
	*/
	void DrawDebugData();
	/**
	* \Get the DebugDraw(), it's use to draw from outside world.cpp
	*/
	p2Draw* GetDebugDraw() const;
	/**
	* \brief Register methods for debug drawing.
	*/
	void SetDebugDraw(p2Draw* debugDraw);
	/**
	* \brief get the contact manager
	*/
	p2ContactManager GetContactManager();
	/**
	* \brief get the gravity
	*/
	p2Vec2 GetGravity() const;

private:
	std::list<p2Body *> m_BodyList;
	p2Vec2 m_Gravity;

	p2Draw* m_DebugDraw = nullptr;

	p2ContactManager m_ContactManager;
};

#endif