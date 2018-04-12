#include "..\..\include\graphics\debug_draw.h"
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

#include <physics/physics.h>
#include <iostream>

#define _USE_MATH_DEFINES

#include <math.h>

namespace sfge 
{
DebugDraw::DebugDraw(sf::RenderWindow* window)
{
	m_Window = window;
}

DebugDraw::~DebugDraw()
{
}

void DebugDraw::DrawRect(p2Vec2 position, float angle, p2Vec2 size, p2Color& color)
{
	sf::RectangleShape rect(meter2pixel(size));
	rect.setOrigin(meter2pixel(size / 2.0f));
	rect.setPosition(meter2pixel(position) + meter2pixel(size / 2.0f));
	rect.setRotation((angle / M_PI) * 180.0f);

	rect.setFillColor(sf::Color(0, 0, 0, 0));
	rect.setOutlineThickness(1.0f);
	rect.setOutlineColor(sf::Color(0, 50, 0, 255));

	m_Window->draw(rect);
}

void DebugDraw::DrawRectFilled(p2Vec2 position, float angle, p2Vec2 size, p2Color& color)
{
	sf::RectangleShape rect(meter2pixel(size));
	rect.setOrigin(meter2pixel(size / 2.0f));
	rect.setPosition(meter2pixel(position) + meter2pixel(size / 2.0f));
	rect.setRotation((angle / M_PI) * 180.0f);

	rect.setFillColor(sf::Color(0, 0, 100, 100));
	rect.setOutlineThickness(1.0f);
	rect.setOutlineColor(sf::Color(0, 50, 0, 255));

	m_Window->draw(rect);
}

void DebugDraw::DrawCircle(p2Vec2 center, float radius, p2Color& color)
{
	sf::CircleShape circle(meter2pixel(radius));
	circle.setOrigin(sf::Vector2f(meter2pixel(radius), meter2pixel(radius)));
	circle.setPosition(meter2pixel(center));

	circle.setFillColor(sf::Color(0, 0, 0, 0));
	circle.setOutlineThickness(1.0f);
	circle.setOutlineColor(sf::Color(0, 50, 0, 255));

	m_Window->draw(circle);
}

void DebugDraw::DrawCircleFilled(p2Vec2 center, float radius, p2Color& color)
{
	sf::CircleShape circle(meter2pixel(radius));
	circle.setOrigin(sf::Vector2f(meter2pixel(radius), meter2pixel(radius)));
	circle.setPosition(meter2pixel(center));

	circle.setFillColor(sf::Color(0, 100, 0, 100));
	circle.setOutlineThickness(1.0f);
	circle.setOutlineColor(sf::Color(0, 50, 0, 255));

	m_Window->draw(circle);
}

void DebugDraw::DrawPolygon(p2Vec2 vertices[], int verticesCount, p2Color& color)
{
	sf::ConvexShape polygon(verticesCount);

	for (int i = 0; i < verticesCount; i++) {
		polygon.setPoint(i, meter2pixel(vertices[i]));
	}
	
	polygon.setFillColor(sf::Color(0, 0, 0, 0));
	polygon.setOutlineThickness(1.0f);
	polygon.setOutlineColor(sf::Color(100, 0, 0, 255));

	m_Window->draw(polygon);
}

void DebugDraw::DrawPolygonFilled(p2Vec2 vertices[], int verticesCount, p2Color& color)
{
	sf::ConvexShape polygon(verticesCount);

	for (int i = 0; i < verticesCount; i++) {
		polygon.setPoint(i, meter2pixel(vertices[i]));
	}

	polygon.setFillColor(p2Color2SfColor(color));
	polygon.setOutlineThickness(1.0f);
	polygon.setOutlineColor(p2Color2SfColor(color / 2));

	m_Window->draw(polygon);
}

void DebugDraw::DrawTransform(p2Vec2 transform)
{
}

sf::Color DebugDraw::p2Color2SfColor(const p2Color & color)
{
	sf::Color result((sf::Uint8)(color.r), (sf::Uint8)(color.g), (sf::Uint8)(color.b), (sf::Uint8)(color.a)); // TO DO
	return result;
}

}
