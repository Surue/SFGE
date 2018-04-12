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

#ifndef SFGE_P2DRAW_H
#define SFGE_P2DRAW_H

#include <p2vector.h>
#include <p2aabb.h>

#include <cstdint>

struct p2Color {
	p2Color() {}

	p2Color(uint32_t red, uint32_t green, uint32_t blue, uint32_t alpha = 1.0f) {
		r = red;
		g = green;
		b = blue;
		a = alpha;
	}

	p2Color operator /(float f) {
		return p2Color(r / f, g / f, b / f, a);
	}

	void Set(uint32_t red, uint32_t green, uint32_t blue, uint32_t alpha = 1.0f) {
		r = red;
		g = green;
		b = blue;
		a = alpha;
	}

	uint32_t r;
	uint32_t g;
	uint32_t b;
	uint32_t a = 255;
};

class p2Draw
{
public:
	p2Draw();

	virtual ~p2Draw() {}

	enum 
	{
		colliderBit			= 0x0001,
		aabbBit				= 0x0002,
		aabbColliderBit		= 0x0004,
		centerOfMassBit		= 0x0008
	};

	/**
	* \brief Set what will be drawn (via flags)
	*/
	void SetFlags(uint32_t flags); 

	/**
	* \brief Get what will be draw (via flags)
	*/
	uint32_t GetFlags();

	virtual void DrawRect(p2Vec2 position, float angle, p2Vec2 size, p2Color& color) = 0;

	virtual void DrawRectFilled(p2Vec2 position, float angle, p2Vec2 size, p2Color& color) = 0;

	virtual void DrawCircle(p2Vec2 center, float radius, p2Color& color) = 0;

	virtual void DrawCircleFilled(p2Vec2 center, float radius, p2Color& color) = 0;

	virtual void DrawPolygon(p2Vec2 vertices[], int verticesCount, p2Color& color) = 0;

	virtual void DrawPolygonFilled(p2Vec2 vertices[], int verticesCount, p2Color& color) = 0;

	virtual void DrawTransform(p2Vec2 transform) = 0;

	p2Color TRANSPARENT = p2Color(0, 0, 0, 0);

protected:
	uint32_t m_DrawFlags;
};

#endif