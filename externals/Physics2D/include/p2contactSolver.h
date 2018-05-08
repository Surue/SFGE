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

struct p2Edge {
	p2Vec2 max;
	p2Vec2 pointA;
	p2Vec2 pointB;
	p2Vec2 vector;
};

class SAT{
public:
	static bool CheckCollisionSAT(p2Contact* contact, p2Manifold& manifold);

	static bool CheckCollisionCircles(p2Contact* contact, p2Manifold& manifold);

	static bool CheckCollisionPolygons(p2Contact* contact, p2Manifold& manifold);

	static bool CheckCollisionPolygonCircle(p2Contact* contact, p2Manifold& manifold);

	static bool CheckCollisionLineCircle(p2Contact* contact, p2Manifold& manifold);

	static bool CheckCollisionLinePolygon(p2Contact* contact, p2Manifold& manifold);

	static p2Vec2 FindContactPoint(const p2Contact* contact, const p2Manifold& manifold);

	static p2Edge FindClosestEdge(const std::vector<p2Vec2> vertices, const p2Vec2 normal);

	static std::vector<p2Vec2> ClipPoints(p2Vec2 pointsA, p2Vec2 pointsB, p2Vec2 normal, float proj);
private:
	SAT();

	static p2Vec2 GetMinMaxProj(p2Vec2 proj[], int sizeArray, p2Vec2 axis);

	static p2Vec2 GetMinMaxProj(std::vector<p2Vec2>& proj, p2Vec2 axis);
};
#endif