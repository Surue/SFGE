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

	//Velocity
	/**
	* \brief get the linear velocity
	*/
	p2Vec2 GetLinearVelocity() const;
	/**
	* \brief set the linear velocity
	*/
	void SetLinearVelocity(const p2Vec2 velocity);
	/**
	* \biref get the angular velocity
	*/
	float GetAngularVelocity() const;
	/**
	* \brief set the angular velocity
	*/
	void SetAngularVelocity(float angularVelocity);
	
	//Position and angle
	/**
	* \brief get the position of body in world location
	*/
	p2Vec2 GetPosition() const;
	/**
	* \brief set the position in the world location
	*/
	void SetPosition(p2Vec2 position);
	/**
	* \brief return angle in radians
	*/
	float GetAngle() const;
	/**
	* \brief set the angle that must be in radians
	*/
	void SetAngle(float angle);

	//Force
	/**
	* \brief add force used to compute acceleration
	*/
	void AddForce(p2Vec2 f);
	/**
	* \brief called at the end of the world step
	*/
	void ClearForce();
	/**
	* \brief return the current force
	*/
	p2Vec2 GetForce();
	/**
	* \brief apply instantanious force  
	*/
	void ApplyImpulse(p2Vec2 impulse, p2Vec2 contactPoint);
	/**
	* \brief the body add as a explosion and add a certain force to all nearby body
	* \param explosiveForce amount of force at the body position
	* \param explosionRadius size of sphere used to get all object touch by the explosion
	* \param explosionPosition can choose the center of explosion
	*/
	void ApplyExplosiveForce(float explosiveForce, float explosionRadius, p2Vec2 explosionPosition);

	/**
	* \brief get the type of body (static, dynamic, kynematic)
	*/
	p2BodyType GetType() const;

	//gravity
	/**
	* \brief get the gravity scale of the body
	*/
	float GetGravityScale() const;
	/**
	* \set the gravity scale
	*/
	void SetGravityScale(float gravityScale);

	//Mass
	/**
	* \brief get the actual mass of body
	*/
	float GetMass() const;
	/**
	* \brief the inverse of mass (used for faster computation)
	*/
	float GetInvMass() const;

	//Colliders
	/**
	* \brief Factory method creating a p2Collider
	* \param colliderDef p2ColliderDef definition of the collider
	* \return p2Collider collider attached to the p2Body
	*/
	p2Collider* CreateCollider(p2ColliderDef* colliderDef);
	/**
	* \brief return all colliders attached to this body
	*/
	std::list<p2Collider*>& GetColliders();

	/**
	* \brief get all shape of the body
	*/
	std::list<p2Shape *>& GetShape();
	
	//AABB
	/**
	* \brief compute the composition of aabb of all collider
	*/
	void ComputeAABB();
	/**
	* \brief return the aabb of body
	*/
	const p2AABB& GetAABB() const;

	/**
	* \brief return the world containing the body
	*/
	p2World* GetWorld();
	/**
	* \brief return the center of mass of the body
	*/
	p2Vec2 GetCentroide();

	//Inertia
	/**
	* \brief return the inversion of the inertia (used for faster computation)
	*/
	float GetInvInertia();

private:
	//Position and rotation
	p2Vec2 m_Position;
	float m_Angle;

	//Movements
	p2Vec2 m_LinearVelocity;
	p2Vec2 m_Force;
	float m_AngularVelocity;

	//Definition of the body
	float m_Inertia;
	float m_InvInertia;
	float m_Mass;
	float m_InvMass;

	p2AABB m_aabb;

	p2BodyType m_Type;

	float m_GravityScale;

	p2World *m_World;

	p2Vec2 m_Centroide;

	std::list<p2Collider*> m_CollidersList;
};

#endif