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


#ifndef SFGE_COLLIDER_H
#define SFGE_COLLIDER_H

#include <engine/component.h>

#include <p2collider.h>


namespace sfge
{

enum class ColliderType
{
	NONE,
	CIRCLE,
	RECTANGLE,
	LINE
};
/**
* \brief Collider Component
*/
class Collider : public Component
{
public:
	using Component::Component;
	/**
	* \brief Initialize the component
	*/
	void Init() override;
	/**
	* \brief Update the component
	*/
	void Update(float dt) override;
	/**
	* \brief Called by the contact listener of the physical world, filter contact to trigger enter or collision enter
	*/
	void OnColliderEnter(Collider* collider);
	/**
	* \brief Called by the contact listener of the physical world, filter contact to trigger exit or collision exit
	*/
	void OnColliderExit(Collider* collider);

	/**
	* brief Factory method to create the Collider component with json arguments given
	*/
	static Collider* LoadCollider(Engine& engine, GameObject* gameObject, json& componentJson);

	void DebugDraw(Engine & engine);

	void DebugDrawAABB(Engine & engine);

protected:
	p2Collider * m_PhysicsCollider = nullptr;
};
}

#endif