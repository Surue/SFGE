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

#ifndef SFGE_P2AABB_H
#define SFGE_P2AABB_H

#include <p2vector.h>

/**
* \brief Struct representing a Axis Aligned Bounding Box
*/
struct p2AABB
{
	p2Vec2 bottomLeft;
	p2Vec2 topRight;

	/**
	* \brief Calculate the center and return it
	*/
	p2Vec2 GetCenter() const;
	/**
	* \brief Calculate the extends and return it
	*/
	const p2Vec2 GetExtends() const;
	/**
	* \brief Check if this contains the given aabb
	*/
	bool Contains(const p2AABB& aabb) const;
	/**
	* \brief Check if this overlap the giver aabb
	*/
	bool Overlap(const p2AABB& aabb) const;
	/**
	* \brief Used by the quadTree to get 4 subdivide aabb
	*/
	void GetSubAABBQuad(p2AABB p2AABB[]) const;
};
#endif // !SFGE_P2AABB:H
