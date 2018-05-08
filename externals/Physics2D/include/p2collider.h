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
	float restitution = 0;
	float friction = 0;
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

	//Shape
	/**
	* \brief Return the shape link to the colliser
	*/
	p2Shape* GetShape() const;
	/**
	* \brief get shape type
	*/
	p2ColliderDef::ShapeType GetShapeType() const;

	//Position
	/**
	* \brief get the offset of the collider to the body
	*/
	p2Vec2& GetOffset();
	/**
	* \brief get the position in world location
	*/
	p2Vec2 GetPosition();

	/**
	* \brief get the body linked to
	*/
	p2Body* GetBody() const;

	/**
	* \brief get the bounciness 
	*/
	float GetRestitution() const;
	/**
	*\brief get the friction
	*/
	float GetFriction() const;
	/**
	* \brief get the centroid of shape
	*/
	p2Vec2 GetCentroide();
	/**
	* \brief get the inertia
	*/
	float GetInertia();


	//Variables
	p2AABB aabb;

private:
	void* m_UserData;
	bool m_IsSensor;
	float m_Restitution;
	float m_Friction;
	p2Shape *m_Shape = nullptr;

	p2ColliderDef::ShapeType m_ShapeType;
	p2Vec2 m_Offset = p2Vec2(0.0f, 0.0f);

	p2Body* m_Body = nullptr;

	p2Vec2 m_Centroide;

	float m_Inertia;
	float m_InvInertia;
};
#endif