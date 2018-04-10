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

namespace sfge 
{
DebugDraw::DebugDraw(sf::RenderWindow* window)
{
	m_Window = window;
}

DebugDraw::~DebugDraw()
{
}

void DebugDraw::DrawRect(p2Vec2 position, p2Vec2 size, p2Color color)
{
	sf::RectangleShape rect(meter2pixel(size));
	rect.setOrigin(meter2pixel(position));
	rect.setFillColor(p2Color2SfColor(p2Draw::TRANSPARENT));
	rect.setOutlineThickness(1.0f);
	rect.setOutlineColor(p2Color2SfColor(color / 2));

	m_Window->draw(rect);
}

void DebugDraw::DrawRectFilled(p2Vec2 position, p2Vec2 size, p2Color color)
{
	sf::RectangleShape rect(meter2pixel(size));
	rect.setOrigin(meter2pixel(position));
	rect.setFillColor(p2Color2SfColor(color));
	rect.setOutlineThickness(1.0f);
	rect.setOutlineColor(p2Color2SfColor(color / 2));

	m_Window->draw(rect);
}

void DebugDraw::DrawCircle(p2Vec2 center, float radius, p2Color color)
{
	sf::CircleShape circle(meter2pixel(radius));
	circle.setOrigin(meter2pixel(center));
	circle.setFillColor(p2Color2SfColor(p2Draw::TRANSPARENT));
	circle.setOutlineThickness(1.0f);
	circle.setOutlineColor(p2Color2SfColor(color));

	m_Window->draw(circle);
}

void DebugDraw::DrawCircleFilled(p2Vec2 center, float radius, p2Color color)
{
	sf::CircleShape circle(meter2pixel(radius));
	circle.setPosition(meter2pixel(center));
	circle.setFillColor(sf::Color(100, 250, 50));
	circle.setOutlineThickness(1.0f);
	circle.setOutlineColor(sf::Color(50, 125, 25));
	m_Window->draw(circle);
}

void DebugDraw::DrawPolygon(p2Vec2 vertices[], int verticesCount, p2Color color)
{
	sf::ConvexShape polygon(verticesCount);

	for (int i = 0; i < verticesCount; i++) {
		polygon.setPoint(i, meter2pixel(vertices[i]));
	}
	
	polygon.setFillColor(p2Color2SfColor(p2Draw::TRANSPARENT));
	polygon.setOutlineThickness(1.0f);
	polygon.setOutlineColor(p2Color2SfColor(color / 2));

	m_Window->draw(polygon);
}

void DebugDraw::DrawPolygonFilled(p2Vec2 vertices[], int verticesCount, p2Color color)
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

sf::Color DebugDraw::p2Color2SfColor(p2Color & color)
{
	sf::Color result((sf::Uint8)color.r % 255, (sf::Uint8)color.g % 255, (sf::Uint8)color.b % 255, (sf::Uint8)color.a % 1);
	return result;
}

}
