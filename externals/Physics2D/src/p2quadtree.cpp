#include "..\include\p2quadtree.h"

#define _CRTDBG_MAP_ALLOC
#include<iostream>
#include <crtdbg.h>
#ifdef _DEBUG
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

#include <p2Contact.h>

p2QuadTree::p2QuadTree()
{
	m_NodeLevel = 0;
	m_Objects = std::list<p2Body*>();
	for (int i = 0; i < CHILD_TREE_NMB; i++) {
		nodes[i] = nullptr;
	}
}

p2QuadTree::p2QuadTree(int nodeLevel, p2AABB aabb)
{
	m_NodeLevel = nodeLevel;
	m_Bounds = aabb;
	m_Objects = std::list<p2Body*>();
	for (int i = 0; i < CHILD_TREE_NMB; i++) {
		nodes[i] = nullptr;
	}
}

p2QuadTree::~p2QuadTree()
{
}

void p2QuadTree::Clear()
{
	m_Objects.clear();
	
	if (nodes[0] != nullptr) {
		for (int i = 0; i < 4; i++) {
			nodes[i]->Clear();
			delete(nodes[i]);
			nodes[i] = nullptr;
		}
	}
}

void p2QuadTree::Split()
{
	p2AABB subAABB[CHILD_TREE_NMB];
	m_Bounds.GetSubAABBQuad(subAABB);

	for (int i = 0; i < CHILD_TREE_NMB; i++) {
		nodes[i] = new p2QuadTree(m_NodeLevel + 1, subAABB[i]);
	}
}

int p2QuadTree::GetIndex(const p2Body* obj) const
{
	p2AABB subAABB[CHILD_TREE_NMB];
	m_Bounds.GetSubAABBQuad(subAABB);
	for (int i = 0; i < CHILD_TREE_NMB; i++) {

		if (subAABB[i].Contains(obj->GetAABB())) {
			return i;
		}
	}

	//If not contains in an aabb => belong to parent
	return -1;
}

void p2QuadTree::Insert(p2Body* obj)
{
	//If the node is already split
	if (nodes[0] != nullptr) {
		int index = GetIndex(obj);
		if (index != -1) {
			nodes[index]->Insert(obj);
			return;
		}
	}
	
	m_Objects.push_back(obj);
	//Check if must split the node
	if (m_Objects.size() > MAX_OBJECTS && m_NodeLevel < MAX_LEVELS) {
		if (nodes[0] == nullptr) {
			Split();
		}

		auto it = m_Objects.begin();
		while (it != m_Objects.end())
		{
			int index = GetIndex(*it);
			if (index != -1) {
				nodes[index]->Insert(*it);
				m_Objects.erase(it++);
			}
			else {
				it++;
			}
		}
	}
}

void p2QuadTree::Retrieve(std::list<p2Contact>& contacts) const
{
	if (nodes[0] == nullptr) {
		if (m_Objects.size() > 1) {
			auto itrObj = m_Objects.begin();
			auto itrObjToCheck = m_Objects.begin();
			
			//Test if two body collide
			while (itrObj != m_Objects.end()) {
				itrObjToCheck = itrObj;
				itrObjToCheck++;

				while (itrObjToCheck != m_Objects.end()) {
					if ((*itrObj)->GetAABB()->Overlap((*itrObjToCheck)->GetAABB())) {
						
						//Test if two collider collide
						for (auto colliderA : (*itrObj)->GetColliders()) {

							for (auto colliderB : (*itrObjToCheck)->GetColliders()) {
								if (colliderA->aabb.Overlap(&colliderB->aabb)) {
									contacts.push_back(p2Contact(colliderA, colliderB));
								}
							}
						}
					}
					itrObjToCheck++;
				}
				itrObj++;
			}
		}
	}
	else {
		//Add children to contacts
		for (int i = 0; i < CHILD_TREE_NMB; i++) {
			nodes[i]->Retrieve(contacts);
		}

		//Retrive all parents objects
		for (auto itrObj = m_Objects.begin(); itrObj != m_Objects.end(); itrObj++) {
			for (int i = 0; i < CHILD_TREE_NMB; i++) {

				for (auto itrObjToCheck = nodes[i]->m_Objects.begin(); itrObjToCheck != nodes[i]->m_Objects.end(); itrObjToCheck++) {
					if ((*itrObj)->GetAABB()->Overlap((*itrObjToCheck)->GetAABB())) {
			
						//Test if two collider collide
						for (auto colliderA : (*itrObj)->GetColliders()) {

							for (auto colliderB : (*itrObjToCheck)->GetColliders()) {
								if (colliderA->aabb.Overlap(&colliderB->aabb)) {
									contacts.push_back(p2Contact(colliderA, colliderB));
								}
							}
						}
					}
				}
			}
		}
	}

	return;
}

void p2QuadTree::SetAABB(const p2AABB aabb)
{
	m_Bounds = aabb;
}

void p2QuadTree::Draw(const p2Draw* debugDraw) const
{
	if (nodes[0] != nullptr) {
		for (int i = 0; i < CHILD_TREE_NMB; i++) {
			nodes[i]->Draw(debugDraw);
		}
	}
	else {
		debugDraw->DrawRect(m_Bounds.bottomLeft, 0, m_Bounds.GetExtends() * 2, p2Color(255, 255, 255));
	}
}
