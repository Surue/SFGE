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

#ifndef SFGE_SHAPE_H_
#define SFGE_SHAPE_H_

#include <engine/engine.h>
#include <engine/component.h>
#include <utility/json_utility.h>

//Externals
#include <SFML/Graphics.hpp>

namespace sfge
{

enum class ShapeType
{
	NONE,
	CIRCLE,
	RECTANGLE,
	POLYGON,
	CONVEX,
	CIRCLE_COLLIDER,
	BOX_COLLIDER
};

/**
* \brief Graphic representation of a shape as Component
*/
class Shape : public Component, public Offsetable
{
public:
	Shape(GameObject* gameObject);
	void Init() override;
	/**
	* \brief Update the position of the SFML shape to the position  of the GameObject Transform
	*/
	void Update(float time) override;
	/**
	* \brief Draw  the SFML shape to the screen
	*/
	void Draw(sf::RenderWindow& window);
	/**
	* \brief Change the Fill Color of the shape
	*/
	void SetFillColor(sf::Color color);
	/**
	* \brief Factory method of a Shape Component
	*/
	static Shape* LoadShape(Engine& engine, json& componentJson, GameObject* gameObject);
protected:
	std::shared_ptr<sf::Shape> m_Shape = nullptr;
};

/**
* \brief Graphic representation of a circle as Component
*/
class Circle : public Shape
{
public:
	Circle(GameObject* gameObject,  float radius);
	/**
	* \brief Update the position of the SFML circle shape to the position  of the GameObject Transform
	*/
	void Update(float dt) override;
	/**
	* \brief Factory method of a Circle Component
	*/
	static Circle* LoadCircle(json& componentJson, GameObject* gameObject);
protected:
	float m_Radius;
};

/**
* \brief Graphic representation of a rectangle as Component
*/
class Rectangle : public Shape
{
public:
	Rectangle(GameObject* gameObject, sf::Vector2f size);
	/**
	* \brief Update the position of the SFML rectangle shape to the position  of the GameObject Transform
	*/
	void Update(float time) override;
	/**
	* \brief Factory method of a Circle Component
	*/
	static Rectangle* LoadRectangle(json& componentJson, GameObject* gameObject);
protected:
	sf::Vector2f m_Size;

};

class CircleCollider : public Shape
{
public:
	CircleCollider(GameObject* gameObject, float radius);
	/**
	* \brief Update the position of the SFML circle shape to the position  of the GameObject Transform
	*/
	void Update(float dt) override;
	/**
	* \brief Factory method of a Circle Component
	*/
	static CircleCollider* LoadCircleCollider(json& componentJson, GameObject* gameObject);
protected:
	float m_Radius;
};

class BoxCollider : public Shape
{
public:
	BoxCollider(GameObject* gameObject, sf::Vector2f size);
	/**
	* \brief Update the position of the SFML circle shape to the position  of the GameObject Transform
	*/
	void Update(float dt) override;
	/**
	* \brief Factory method of a Circle Component
	*/
	static BoxCollider* LoadBoxCollider(json& componentJson, GameObject* gameObject);
protected:
	sf::Vector2f m_Size;
};

/**
* \brief Graphic Manager part loading and managing the Shapes components
*/
class ShapeManager
{

public:
	ShapeManager(GraphicsManager& graphicsManager);
	void AddShape(Shape* shape);
	/**
	* \brief Draw all the shapes
	*/
	void Draw(sf::RenderWindow& window);
	/**
	* \brief Called before the new Scene is loaded
	*/
	void Reset();
	/**
	* \brief Called at the end of the loading frame
	*/
	void Reload();
protected:
	std::list<Shape*> m_Shapes;
	GraphicsManager& m_GraphicsManager;
};

}
#endif /* SRC_GRAPHICS_SHAPE_H_ */
