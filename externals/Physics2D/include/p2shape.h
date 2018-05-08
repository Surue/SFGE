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

#ifndef SFGE_P2SHAPE_H
#define SFGE_P2SHAPE_H

#include <p2vector.h>
#include <p2aabb.h>

#include <string>
#include <vector>

/**
* \brief Abstract representation of a shape
*/
class p2Shape
{
public:
	/**
	* \brief Clone shape
	*/
	virtual p2Shape* Clone() const = 0;

	virtual void ComputeAABB(p2AABB* aabb, p2Vec2 position, float angle) const = 0;

	virtual void GetNormals(p2Vec2 normals[], p2Vec2 vectors[], int numOfVectors) const;
	virtual void GetNormals(std::vector<p2Vec2>& normals, std::vector<p2Vec2> vectors) const;
};

/**
* \brief Representation of a circle
*/
class p2CircleShape : public p2Shape
{
public:
	/**
	* \brief Clone shape
	*/
	p2Shape * Clone() const override;

	//Radius
	/**
	* \brief Set the radius
	*/
	void SetRadius(float radius);
	/**
	* \brief Get the radius
	*/
	float GetRadius();

	/**
	* \brief Compute AABB
	*/
	void ComputeAABB(p2AABB* aabb, p2Vec2 position, float angle) const override;
private:
	float m_Radius;
};

/**
* \brief Representation of a circle
*/
class p2PolygonShape : public p2Shape
{
public:
	/**
	* \brief Clone shape
	*/
	p2Shape * Clone() const override;
	/**
	* \brief Compute AABB
	*/
	void ComputeAABB(p2AABB* aabb, p2Vec2 position, float angle) const override;

	//Vertices
	/**
	* \brief set number of vertex
	*/
	void SetVerticesCount(int verticesCount);
	/**
	* \brief get number of vertex
	*/
	int GetVerticesCount();
	/**
	* \brief set one vertex
	* \param vertice
	* \param index of vertex
	*/
	void SetVertice(p2Vec2 vertice, int index);
	/**
	* \brief get vertex at the given index
	*/
	const p2Vec2 GetVertice(int index) const;
	/**
	* \brief get all vertices in local position
	*/
	const std::vector<p2Vec2> GetVertices() const;
	/**
	* \brief get all vertices in world position
	*/
	const std::vector<p2Vec2> GetVerticesWorld(p2Vec2 position, float angle) const;
	
	/**
	* \brief get edges
	* \param vectors std::vectors used as callback
	*/
	void GetVectorsVertices(std::vector<p2Vec2>& vectors, p2Vec2 position, float angle);
	/**
	* \brief get vectors from center to vertex
	* \param vectors [] used as callback
	*/
	void GetVectorsCenter(p2Vec2 vectors[], p2Vec2 position, float angle);
private:
	std::vector<p2Vec2> m_Vertices;
};

class p2LineShape : public p2Shape
{
public:
	/**
	* \brief Clone shape
	*/
	p2Shape * Clone() const override;
	/**
	* \brief Compute AABB
	*/
	virtual void ComputeAABB(p2AABB* aabb, p2Vec2 position, float angle) const override;
	/**
	* \brief Set line points
	*/
	void SetLine(p2Vec2 p1, p2Vec2 p2);

	p2Vec2 m_PosA, m_PosB;
private:
};

#endif