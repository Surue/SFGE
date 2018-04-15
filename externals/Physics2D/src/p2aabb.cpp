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

#define _CRTDBG_MAP_ALLOC
#include<iostream>
#include <crtdbg.h>
#ifdef _DEBUG
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

#include <p2aabb.h>
#include <cmath>
#include <iostream>

const p2Vec2 p2AABB::GetCenter() const
{
	return (bottomLeft + topRight) * 0.5f;
}

const p2Vec2 p2AABB::GetExtends() const
{
	return (topRight - bottomLeft) * 0.5f;
}

bool p2AABB::Contains(const p2AABB* other) const
{
	bool isContaining = true;

	isContaining = isContaining && bottomLeft.x <= other->bottomLeft.x;
	isContaining = isContaining && bottomLeft.y >= other->bottomLeft.y;
	isContaining = isContaining && topRight.x >= other->topRight.x;
	isContaining = isContaining && topRight.y <= other->topRight.y;

	return isContaining;
}

bool p2AABB::Overlap(const p2AABB* aabb) const
{
	p2Vec2 d1, d2;
	d1 = aabb->bottomLeft - topRight;
	d2 = aabb->topRight * (-1) + bottomLeft;

	if (d1.x > 0.0f || d1.y < 0.0f) {
		return false;
	}

	if (d2.x > 0.0f || d2.y < 0.0f) {
		return false;
	}

	return true;
}

void p2AABB::GetSubAABBQuad(p2AABB p2aabb[]) const
{
	//Top Left
	p2aabb[0].bottomLeft = p2Vec2(bottomLeft.x, bottomLeft.y + GetExtends().y);
	p2aabb[0].topRight = p2Vec2(topRight.x - GetExtends().x, topRight.y);

	//Top Right
	p2aabb[1].bottomLeft = p2aabb[0].bottomLeft + p2Vec2(GetExtends().x, 0);
	p2aabb[1].topRight = topRight;

	//Bottom Left
	p2aabb[2].bottomLeft = bottomLeft;
	p2aabb[2].topRight = p2aabb[0].topRight + p2Vec2(0, -GetExtends().y);

	//Bottom Right
	p2aabb[3].bottomLeft = p2aabb[0].bottomLeft + p2Vec2(GetExtends().x, -GetExtends().y);
	p2aabb[3].topRight = p2aabb[0].topRight + p2Vec2(GetExtends().x, -GetExtends().y);
}
