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

#ifndef DEBUGDRAW_H
#define DEBUGDRAW_H

#include <p2physics.h>

//Externals
#include <SFML/Graphics.hpp>

namespace sfge {

class DebugDraw : public p2Draw
{
public:
	DebugDraw(sf::RenderWindow *window);
	virtual ~DebugDraw();

	void DrawRect(const p2Vec2 position, float angle, const p2Vec2 size,p2Color color) const;

	void DrawRectFilled(const p2Vec2 position, float angle, const p2Vec2 size, p2Color color) const;

	void DrawCircle(const p2Vec2 center, float radius, p2Color color) const;

	void DrawCircleFilled(const p2Vec2 center, float radius, p2Color color) const;

	void DrawPolygon(const std::vector<p2Vec2> vertices, p2Color color) const;

	void DrawPolygonFilled(const std::vector<p2Vec2> vertices, p2Color color) const;

	void DrawTransform(const p2Vec2 transform) const;

	void DrawLine(const p2Vec2 posA, const p2Vec2 posB) const;

	sf::Color p2Color2SfColor(p2Color color) const;
private:
	sf::RenderWindow *m_Window;
};
}
#endif // !DEBUGDRAW_H
