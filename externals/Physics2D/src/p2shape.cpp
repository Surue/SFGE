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

#include <p2shape.h>
#include <p2matrix.h>
#include <iostream>

p2Shape * p2CircleShape::Clone() const
{
	p2CircleShape* clone = new p2CircleShape();
	*clone = *this;
	return clone;
}

void p2CircleShape::SetRadius(float radius)
{
	m_Radius = radius;
}

float p2CircleShape::GetRadius()
{
	return m_Radius;
}

void p2CircleShape::ComputeAABB(p2AABB * aabb, p2Vec2 position, float angle) const
{
	aabb->bottomLeft = position - p2Vec2(m_Radius, -m_Radius);
	aabb->topRight = position + p2Vec2(m_Radius, -m_Radius);
}

std::string p2CircleShape::GetJson() const
{
	return "\"name\" : \"CircleColliderShape\", \"type\" : 3, \"shape_type\" : 5, \"radius\" : " + std::to_string(m_Radius * 100.0f);
}

p2Shape * p2RectShape::Clone() const
{
	p2RectShape* clone = new p2RectShape();
	*clone = *this;
	return clone;
}

void p2RectShape::SetSize(p2Vec2 size)
{
	m_Size = size;
}

p2Vec2 p2RectShape::GetSize()
{
	return m_Size;
}

void p2RectShape::ComputeAABB(p2AABB* aabb, p2Vec2 position, float angle) const
{
	p2Vec2 topRight = (p2Mat22::RotationMatrix(angle) * p2Vec2(m_Size.x / 2.0f, m_Size.y / 2.0f));
	p2Vec2 bottomLeft = (p2Mat22::RotationMatrix(angle) * p2Vec2(m_Size.x / 2.0f, -m_Size.y / 2.0f));
	
	aabb->bottomLeft = p2Vec2(position.x - std::fmaxf(std::abs(topRight.x), std::abs(bottomLeft.x)),
		                      position.y + std::fmaxf(std::abs(topRight.y), std::abs(bottomLeft.y)));
	aabb->topRight = p2Vec2(position.x + std::fmaxf(std::abs(topRight.x), std::abs(bottomLeft.x)),
							position.y - std::fmaxf(std::abs(topRight.y), std::abs(bottomLeft.y)));
}

std::string p2RectShape::GetJson() const
{
	return "\"name\" : \"BoxColliderShape\", \"type\" : 3, \"shape_type\" : 6, \"size\" : [" + std::to_string(m_Size.x * 100.0f) + "," + std::to_string(m_Size.y * 100.0f) + "]";
}
