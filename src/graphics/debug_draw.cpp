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

void DebugDraw::DrawRect(const p2Vec2 position, float angle, const p2Vec2 size, p2Color& color) const
{
	sf::RectangleShape rect(meter2pixel(size));
	rect.setOrigin(meter2pixel(size / 2.0f));
	rect.setPosition(meter2pixel(position) + meter2pixel(size / 2.0f));
	rect.setRotation((angle / M_PI) * 180.0f);

	rect.setFillColor(sf::Color(0, 0, 0, 0));
	rect.setOutlineThickness(1.0f);
	rect.setOutlineColor(p2Color2SfColor(color / 2));

	m_Window->draw(rect);
}

void DebugDraw::DrawRectFilled(const p2Vec2 position, float angle, const p2Vec2 size, p2Color& color) const
{
	sf::RectangleShape rect(meter2pixel(size));
	rect.setOrigin(meter2pixel(size / 2.0f));
	rect.setPosition(meter2pixel(position) + meter2pixel(size / 2.0f));
	rect.setRotation((angle / M_PI) * 180.0f);

	rect.setFillColor(p2Color2SfColor(color.Set(color.r, color.g, color.b, 100)));
	rect.setOutlineThickness(1.0f);
	rect.setOutlineColor(p2Color2SfColor(color / 2));

	m_Window->draw(rect);
}

void DebugDraw::DrawCircle(const p2Vec2 center, float radius, p2Color& color) const
{
	sf::CircleShape circle(meter2pixel(radius));
	circle.setOrigin(sf::Vector2f(meter2pixel(radius), meter2pixel(radius)));
	circle.setPosition(meter2pixel(center));

	circle.setFillColor(sf::Color(0, 0, 0, 0));
	circle.setOutlineThickness(1.0f);
	circle.setOutlineColor(p2Color2SfColor(color / 2));

	m_Window->draw(circle);
}

void DebugDraw::DrawCircleFilled(const p2Vec2 center, float radius, p2Color& color) const
{
	sf::CircleShape circle(meter2pixel(radius));
	circle.setOrigin(sf::Vector2f(meter2pixel(radius), meter2pixel(radius)));
	circle.setPosition(meter2pixel(center));

	circle.setFillColor(p2Color2SfColor(color.Set(color.r, color.g, color.b, 100)));
	circle.setOutlineThickness(1.0f);
	circle.setOutlineColor(p2Color2SfColor(color / 2));

	m_Window->draw(circle);
}

void DebugDraw::DrawPolygon(const std::vector<p2Vec2> vertices, p2Color& color) const
{
	sf::ConvexShape polygon(vertices.size());

	for (int i = 0; i < vertices.size(); i++) {
		polygon.setPoint(i, meter2pixel(vertices[i]));
	}
	
	polygon.setFillColor(sf::Color(0, 0, 0, 0));
	polygon.setOutlineThickness(1.0f);
	polygon.setOutlineColor(p2Color2SfColor(color / 2));

	m_Window->draw(polygon);
}

void DebugDraw::DrawPolygonFilled(const std::vector<p2Vec2> vertices, p2Color& color) const
{
	sf::ConvexShape polygon(vertices.size());

	for (int i = 0; i < vertices.size(); i++) {
		polygon.setPoint(i, meter2pixel(vertices[i]));
	}

	polygon.setFillColor(p2Color2SfColor(color.Set(color.r, color.g, color.b, 100)));
	polygon.setOutlineThickness(1.0f);
	polygon.setOutlineColor(p2Color2SfColor(color / 2));

	m_Window->draw(polygon);
}

void DebugDraw::DrawTransform(const p2Vec2 transform) const
{
}

void DebugDraw::DrawLine(const p2Vec2 posA, const p2Vec2 posB) const
{
	sf::Vertex line[] = {
		sf::Vertex(sfge::meter2pixel(posA)),
		sf::Vertex(sfge::meter2pixel(posB))
	};

	m_Window->draw(line, 2, sf::Lines);
}

sf::Color DebugDraw::p2Color2SfColor(const p2Color & color) const
{
	sf::Color result((sf::Uint8)(color.r), (sf::Uint8)(color.g), (sf::Uint8)(color.b), (sf::Uint8)(color.a)); // TO DO
	return result;
}

}
