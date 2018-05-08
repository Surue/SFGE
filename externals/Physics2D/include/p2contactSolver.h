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

#ifndef SFGE_P2CONTACT_SOLVER_H
#define SFGE_P2CONTACT_SOLVER_H

#include <p2body.h>

class p2Contact;

/**
* \brief representation of collision
*/
struct p2Manifold {
	p2Body *bodyA;
	p2Body *bodyB;
	float penetration;
	p2Vec2 normal;

	p2Vec2 contactPoint; 

	bool ShouldResolve = false;

	float normalImpulse = 0.0f;
	float tangentImpulse = 0.0f;
};

/**
* \brief represent an edge, use to find contact point
*/
struct p2Edge {
	p2Vec2 max;
	p2Vec2 pointA;
	p2Vec2 pointB;
	p2Vec2 vector;
};

class ContactSolver{
public:
	/**
	* \brief is called from the p2Contact::Update to check the collision and create the manifold
	*/
	static bool CheckCollision(p2Contact* contact, p2Manifold& manifold);
	/**
	* \brief check collision between 2 circles
	*/
	static bool CollisionCircles(p2Contact* contact, p2Manifold& manifold);
	/**
	* \brief check collision between 2 polygons
	*/
	static bool CollisionPolygons(p2Contact* contact, p2Manifold& manifold);
	/**
	* \brief check collision between a circle and a polygon
	*/
	static bool CollisionPolygonCircle(p2Contact* contact, p2Manifold& manifold);

	/**
	* \brief check collision between a circle and a line
	*/
	static bool CollisionLineCircle(p2Contact* contact, p2Manifold& manifold);
	/**
	* \brief check collision between a line and a polygon
	*/
	static bool CollisionLinePolygon(p2Contact* contact, p2Manifold& manifold);
	/**
	* \brief find the contact point of a collision
	*/
	static p2Vec2 FindContactPoint(const p2Contact* contact, const p2Manifold& manifold);
	/**
	* \brief closest edge of a polygon
	*/
	static p2Edge FindClosestEdge(const std::vector<p2Vec2> vertices, const p2Vec2 normal);
	/**
	* \brief clip a point onto a line
	*/
	static std::vector<p2Vec2> ClipPoints(p2Vec2 pointsA, p2Vec2 pointsB, p2Vec2 normal, float proj);
private:
	//Make it non instanciable
	ContactSolver();

	static p2Vec2 GetMinMaxProj(p2Vec2 proj[], int sizeArray, p2Vec2 axis);

	static p2Vec2 GetMinMaxProj(std::vector<p2Vec2>& proj, p2Vec2 axis);
};
#endif