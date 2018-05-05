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

#ifndef SFGE_BODY2D_H
#define SFGE_BODY2D_G

#include <engine/log.h>
#include <engine/component.h>
#include <utility/json_utility.h>

#include <p2body.h>

namespace sfge
{
class Collider;

/**
* \brief Rigidbody Component
*/
class Body2d : public Component
{
public:
	using Component::Component;
	/**
	* \brief Initialisation of the Component
	*/
	void Init() override;
	/**
	* \brief Callled every frame, update the Transform of the parent GameObject
	*/
	void Update(float dt) override;
	/**
	* \brief Getter of the Physics2D body
	* \return the Physics2D of the the Body2d Component
	*/
	p2Body* GetBody();

	void SetEulerAngle(float eulerAngle);

	void SetVelocity(p2Vec2 v);
	p2Vec2 GetVelocity();

	void SetPosition(sf::Vector2f pos);

	float GetGravityScale() const;
	void SetGravityScale(float gravityScale);

	void AddExplosiveForce(float explosiveForce, float explosionRadius, sf::Vector2f explosionPosition) const;

	/**
	* \brief Factory method that creates a Body2d Component
	* \return Body2d ptr that will be attached to the GameObject and will be destroyed by the PhysicsManager
	*/
	static Body2d* LoadBody2d(Engine& engine, GameObject* gameObject, json& componentJson);

protected:
	p2Body * m_Body = nullptr;
	std::list<Collider*> m_Colliders;
};

}

#endif