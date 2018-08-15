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

#if WIN32
#define _CRTDBG_MAP_ALLOC
#include<iostream>
#include <crtdbg.h>
#ifdef _DEBUG
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif
#endif

#include <p2aabb.h>
#include <cmath>
#include <iostream>

p2Vec2 p2AABB::GetCenter() const
{
	return p2Vec2((bottomLeft.x + topRight.x), (topRight.y + bottomLeft.y)) * 0.5f;
}

const p2Vec2 p2AABB::GetExtends() const
{
	return (topRight - bottomLeft) * 0.5f;
}

bool p2AABB::Contains(const p2AABB& aabb) const
{
	bool isContaining = true;
	isContaining = isContaining && bottomLeft.x <= aabb.bottomLeft.x;
	isContaining = isContaining && bottomLeft.y >= aabb.bottomLeft.y;
	isContaining = isContaining && topRight.x >= aabb.topRight.x;
	isContaining = isContaining && topRight.y <= aabb.topRight.y;

	return isContaining;
}

bool p2AABB::Overlap(const p2AABB& aabb) const
{
	bool x = (std::abs(GetCenter().x - aabb.GetCenter().x) <= GetExtends().x  + aabb.GetExtends().x );
	bool y = (std::abs(GetCenter().y - aabb.GetCenter().y) <= GetExtends().y * -1 + aabb.GetExtends().y * -1);

	return x && y;
}

void p2AABB::GetSubAABBQuad(p2AABB p2aabb[]) const
{
	p2Vec2 extends = GetExtends();

	//Top Left
	p2aabb[0].bottomLeft = p2Vec2(bottomLeft.x, bottomLeft.y + extends.y);
	p2aabb[0].topRight = p2Vec2(topRight.x - extends.x, topRight.y);

	//Top Right
	p2aabb[1].bottomLeft = p2Vec2(bottomLeft.x + extends.x, bottomLeft.y + extends.y);
	p2aabb[1].topRight = topRight;

	//Bottom Left
	p2aabb[2].bottomLeft = bottomLeft;
	p2aabb[2].topRight = p2Vec2(topRight.x - extends.x, topRight.y - extends.y);

	//Bottom Right
	p2aabb[3].bottomLeft = p2Vec2(bottomLeft.x + extends.x, bottomLeft.y);
	p2aabb[3].topRight = p2Vec2(topRight.x, topRight.y - extends.y);
}
