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

#include <p2shape.h>
#include <p2matrix.h>
#include <iostream>
#include <algorithm>
#include <cmath>

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

void p2Shape::GetNormals(p2Vec2 normals[], p2Vec2 vectors[], int numOfVectors) const
{
	for (int i = 0; i < numOfVectors; i++) {
		normals[i] = vectors[i].Normal().Normalized();
	}
}

void p2Shape::GetNormals(std::vector<p2Vec2>& normals, std::vector<p2Vec2> vectors) const
{
	for (int i = 0; i < vectors.size(); i++) {
		normals[i] = vectors[i].Normal().Normalized();
	}
}

p2Shape * p2PolygonShape::Clone() const
{
	p2PolygonShape* clone = new p2PolygonShape();
	*clone = *this;
	return clone;
}

void p2PolygonShape::ComputeAABB(p2AABB * aabb, p2Vec2 position, float angle) const
{
	float minX, maxX, minY, maxY;
	minX = ((p2Mat22::RotationMatrix(-angle) * m_Vertices[0]) + position).x;
	maxX = minX;

	minY = ((p2Mat22::RotationMatrix(-angle) * m_Vertices[0]) + position).y;
	maxY = minY;

	for (int i = 1; i < m_Vertices.size(); i++) {
		p2Vec2 pos = (p2Mat22::RotationMatrix(-angle) * m_Vertices[i]) + position;
		if (pos.x > maxX) maxX = pos.x;
		if (pos.x < minX) minX = pos.x;
		if (pos.y > maxY) maxY = pos.y;
		if (pos.y < minY) minY = pos.y;
	}

	aabb->bottomLeft = p2Vec2(minX, maxY);
	aabb->topRight = p2Vec2(maxX, minY);
}

void p2PolygonShape::SetVerticesCount(int verticesCount)
{
	m_Vertices.resize(verticesCount);
}

int p2PolygonShape::GetVerticesCount()
{
	return m_Vertices.size();
}

void p2PolygonShape::SetVertice(p2Vec2 vertice, int index)
{
	m_Vertices[index] = vertice;

	//Check if counter-clockwise, if not inverse order of verices
	if (index == m_Vertices.size() - 1) {
		float sum = 0;
		int i = 0;
		for (i = 0; i < m_Vertices.size() - 2; ++i) {
			sum += ((m_Vertices[i + 1].x - m_Vertices[i].x)*(m_Vertices[i + 1].y + m_Vertices[i].y));
		}

		sum += ((m_Vertices[i + 1].x - m_Vertices[0].x)*(m_Vertices[i + 1].y + m_Vertices[0].y));

		if (sum < 0) {
			std::cout << "clockwise\n";
			std::reverse(m_Vertices.begin(), m_Vertices.end());
		}
	}
}

const p2Vec2 p2PolygonShape::GetVertice(int index) const
{
	return m_Vertices[index];
}

const std::vector<p2Vec2> p2PolygonShape::GetVertices() const
{
	return m_Vertices;
}

const std::vector<p2Vec2> p2PolygonShape::GetVerticesWorld(p2Vec2 position, float angle) const
{
	std::vector<p2Vec2> worldCoords;
	worldCoords.resize(m_Vertices.size());

	for (int i = 0; i < m_Vertices.size(); i++) {
		worldCoords[i] = (p2Mat22::RotationMatrix(-angle) * m_Vertices[i]) + position;
	}

	return worldCoords;
}

void p2PolygonShape::GetVectorsVertices(std::vector<p2Vec2>& vectors, p2Vec2 position, float angle)
{
	for (int i = 0; i < m_Vertices.size() - 1; i++) {
		vectors[i] = (p2Mat22::RotationMatrix(-angle) * m_Vertices[(i + 1)]) + position - ((p2Mat22::RotationMatrix(-angle) * m_Vertices[i]) + position);
	}

	vectors[m_Vertices.size()-1] = (p2Mat22::RotationMatrix(-angle) * m_Vertices[0]) + position - ((p2Mat22::RotationMatrix(-angle) * m_Vertices[m_Vertices.size() - 1]) + position);
}

void p2PolygonShape::GetVectorsCenter(p2Vec2 vectors[], p2Vec2 position, float angle)
{
}

p2Shape * p2LineShape::Clone() const
{
	p2LineShape* clone = new p2LineShape();
	*clone = *this;
	return clone;
}

void p2LineShape::ComputeAABB(p2AABB * aabb, p2Vec2 position, float angle) const
{
	aabb->bottomLeft.x = std::fmin(m_PosA.x, m_PosB.x);
	aabb->bottomLeft.y = std::fmax(m_PosA.y, m_PosB.y);

	aabb->topRight.x = std::fmax(m_PosA.x, m_PosB.x);
	aabb->topRight.y = std::fmin(m_PosA.y, m_PosB.y);
}

void p2LineShape::SetLine(p2Vec2 p1, p2Vec2 p2)
{
	m_PosA = p1;
	m_PosB = p2;
}
