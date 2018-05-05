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

#ifndef SFGE_P2BODY_H
#define SFGE_P2BODY_H

#include <p2aabb.h>
#include <p2shape.h>
#include <list>

class p2World;
class p2Collider;
struct p2ColliderDef;

enum class p2BodyType
{
	STATIC,
	KINEMATIC,
	DYNAMIC
};

/**
* \brief Struct Defining the p2Body when creating it
*/
struct p2BodyDef
{
	p2BodyType type;
	p2Vec2 position;
	p2Vec2 linearVelocity;
	p2Vec2 InitialForce;
	float gravityScale = 1;
	float mass = 1;
	float angle = 0;
};

/**
* \brief Rigidbody representation
*/
class p2Body
{
public:
	p2Body();
	p2Body(p2BodyDef bodyDef, p2World* world);

	~p2Body();

	p2Vec2 GetLinearVelocity() const;
	
	void SetLinearVelocity(p2Vec2& velocity);

	float GetAngularVelocity() const;
	void SetAngularVelocity(float angularVelocity);
	
	p2Vec2 GetPosition() const;
	void SetPosition(p2Vec2 position);

	/**
	* \brief return angle in radians
	*/
	float GetAngle() const;
	/**
	* \brief the angle must be in radians
	*/
	void SetAngle(float angle);

	void AddForce(p2Vec2 f);
	void ClearForce();
	p2Vec2 GetForce();

	void ApplyImpulse(p2Vec2 impulse, p2Vec2 contactPoint);

	p2BodyType GetType() const;
	float GetGravityScale() const;
	void SetGravityScale(float gravityScale);
	float GetMass() const;
	float GetInvMass() const;
	/**
	* \brief Factory method creating a p2Collider
	* \param colliderDef p2ColliderDef definition of the collider
	* \return p2Collider collider attached to the p2Body
	*/
	p2Collider* CreateCollider(p2ColliderDef* colliderDef);

	std::list<p2Collider*>& GetColliders();

	std::list<p2Shape *>& GetShape();
	
	void ComputeAABB();

	const p2AABB* GetAABB() const;

	p2World* GetWorld();

	p2Vec2 GetCentroide();

	float GetInvInertia();

private:
	p2Vec2 position;
	p2Vec2 linearVelocity;
	p2Vec2 m_Force;

	p2AABB aabb;

	float angularVelocity;
	p2BodyType type;
	float gravityScale;
	float m_Mass;
	float m_InvMass;

	float m_Angle;

	float m_Inertia;
	float m_InvInertia;

	p2World *world;

	p2Vec2 m_Centroide;

	std::list<p2Collider*> m_CollidersList;
};

#endif