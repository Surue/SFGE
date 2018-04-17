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


#ifndef SFGE_P2COLLIDER_H
#define SFGE_P2COLLIDER_H

#include <p2shape.h>
#include <p2aabb.h>

class p2Body;
/**
* \brief Struct defining a p2Collider when creating one
*/
struct p2ColliderDef
{
	void* userData;
	p2Shape* shape;
	float restitution;
	bool isSensor;
	p2Vec2 offset = p2Vec2(0.0f, 0.0f);
	enum ShapeType{
		CIRCLE,
		RECT,
		POLYGON
	};

	ShapeType shapeType;
};

/**
* \brief Representation of a Collider attached to a p2Body
*/

class p2Collider
{
public:
	p2Collider();
	p2Collider(p2ColliderDef colliderDef, p2Body* body);

	~p2Collider();

	/**
	* \brief Check if the p2Collider is a sensor
	*/
	bool IsSensor() const;
	/**
	* \brief Return the userData
	*/
	void* GetUserData() const;

	p2Shape* GetShape() const;

	p2Vec2& GetOffset();

	p2Vec2 GetPosition();

	p2ColliderDef::ShapeType GetShapeType() const;

	p2Body* GetBody() const;

	p2AABB aabb;

private:
	void* userData;
	bool isSensor;
	float restitution;
	p2Shape *shape = nullptr;

	p2ColliderDef::ShapeType shapeType;
	p2Vec2 m_Offset = p2Vec2(0.0f, 0.0f);

	p2Body* m_Body = nullptr;
};


#endif