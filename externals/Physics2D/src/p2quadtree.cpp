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

int p2QuadTree::GetIndex(p2Body * obj)
{
	p2AABB subAABB[CHILD_TREE_NMB];
	m_Bounds.GetSubAABBQuad(subAABB);
	for (int i = 0; i < CHILD_TREE_NMB; i++) {

		if (subAABB[i].Contains(obj->aabb)) {
			return i;
		}
	}

	//If not contains in an aabb => belong to parent
	return -1;
}

void p2QuadTree::Insert(p2Body * obj)
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

std::list<p2Contact> p2QuadTree::Retrieve()
{
	std::list<p2Contact> contacts;
	if (nodes[0] == nullptr) {
		if (m_Objects.size() > 1) {
			auto itrObj = m_Objects.begin();
			auto itrObjToCheck = m_Objects.begin();
			
			//Test if two body collide
			while (itrObj != m_Objects.end()) {
				itrObjToCheck = itrObj;
				itrObjToCheck++;

				while (itrObjToCheck != m_Objects.end()) {
					if ((*itrObj)->aabb.Overlap((*itrObjToCheck)->aabb)) {
						
						//Test if two collider collide
						for (auto colliderA : (*itrObj)->GetColliders()) {

							for (auto colliderB : (*itrObjToCheck)->GetColliders()) {
								if (colliderA->aabb.Overlap(colliderB->aabb)) {
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
			auto childContacts = nodes[i]->Retrieve();
			for (p2Contact contact : childContacts) {
				contacts.push_back(contact);
			}
		}

		//Retrive all parents objects
		for (auto itrObj = m_Objects.begin(); itrObj != m_Objects.end(); itrObj++) {
			for (int i = 0; i < CHILD_TREE_NMB; i++) {

				for (auto itrObjToCheck = nodes[i]->m_Objects.begin(); itrObjToCheck != nodes[i]->m_Objects.end(); itrObjToCheck++) {
					if ((*itrObj)->aabb.Overlap((*itrObjToCheck)->aabb)) {
			
						//Test if two collider collide
						for (auto colliderA : (*itrObj)->GetColliders()) {

							for (auto colliderB : (*itrObjToCheck)->GetColliders()) {
								if (colliderA->aabb.Overlap(colliderB->aabb)) {
									contacts.push_back(p2Contact(colliderA, colliderB));
								}
							}
						}
					}
				}
			}
		}
	}

	return contacts;
}

void p2QuadTree::SetAABB(p2AABB aabb)
{
	m_Bounds = aabb;
}

void p2QuadTree::Draw(p2Draw* debugDraw)
{
	p2Vec2 vertices[4];
	vertices[0] = m_Bounds.bottomLeft;
	vertices[1] = p2Vec2(m_Bounds.topRight.x, m_Bounds.bottomLeft.y);
	vertices[2] = m_Bounds.topRight;
	vertices[3] = p2Vec2(m_Bounds.bottomLeft.x, m_Bounds.topRight.y);
	debugDraw->DrawPolygon(vertices, 4, p2Color(255, 255, 255));

	if (nodes[0] != nullptr) {
		for (int i = 0; i < CHILD_TREE_NMB; i++) {
			nodes[i]->Draw(debugDraw);
		}
	}
}
