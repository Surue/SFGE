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

#ifndef SFGE_P2QUADTREE_H
#define SFGE_P2QUADTREE_H

#include <list>

#include <p2vector.h>
#include <p2aabb.h>
#include <p2body.h>
#include <p2draw.h>

class p2Contact;

/**
* \brief Representation of a tree with 4 branches containing p2Body defined by their p2AABB
*/
class p2QuadTree
{
public:
	p2QuadTree();
	p2QuadTree(int nodeLevel, p2AABB aabb);
	~p2QuadTree();

	/**
	* \brief Remove all objects leafs and quadtrees children
	*/
	void Clear();
	/**
	* \brief Called when node have too much objects and split the current node into four
	*/
	void Split();
	/**
	* \brief Get the index of the child trees of the p2Body
	*/
	int GetIndex(const p2Body* obj) const;
	/**
	* \brief Insert a new p2Body in the tree
	*/
	void Insert(p2Body* obj);
	/**
	* \brief Return a list of all the p2Body that might collide
	*/
	void Retrieve(std::list<p2Contact>& contacts) const;
	/**
	* \brief Return a list of all bodies inside the given aabb
	*/
	std::list<p2Body*> AABBOverlap(p2AABB aabb) const;
	
	//AABB
	/**
	* \brief set the aabb for this quadTree
	*/
	void SetAABB(const p2AABB aabb);
	/**
	* \brief get the aabb
	*/
	p2AABB& GetAABB();

	/**
	* \brief Part of Debug Draw
	*/
	void Draw(const p2Draw* debugDraw) const;
	
private:
	static const int MAX_OBJECTS = 10;
	static const int MAX_LEVELS = 5;
	static const int CHILD_TREE_NMB = 4;

	int m_NodeLevel = 0;
	p2QuadTree* m_Nodes[CHILD_TREE_NMB];
	std::list<p2Body*> m_Objects;
	p2AABB m_Bounds;
};

#endif