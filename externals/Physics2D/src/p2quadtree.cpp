#include "..\include\p2quadtree.h"

p2QuadTree::p2QuadTree(int nodeLevel, p2AABB bounds)
{
}

p2QuadTree::~p2QuadTree()
{
}

void p2QuadTree::Clear()
{
}

void p2QuadTree::Split()
{
}

int p2QuadTree::GetIndex(p2Body * rect)
{
	return 0;
}

void p2QuadTree::Insert(p2Body * obj)
{
}

std::list<p2Contact> p2QuadTree::Retrieve()
{
	std::list<p2Contact> contacts;

	if (nodes[0] == nullptr) {
		if (m_Objects.size() > 1) {
			auto itrObj = m_Objects.begin();
			auto itrObjToCheck = m_Objects.begin();

			while (itrObj != m_Objects.end()) {
				itrObjToCheck = itrObj;
				itrObjToCheck++;

				while (itrObjToCheck != m_Objects.end()) {
					if ((*itrObj)->aabb.Contains((*itrObjToCheck)->aabb)) {
						//contacts.push_back(p2Contact(*itrObj, *itrObjToCheck)); TO DO
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
			contacts.insert(contacts.end(), childContacts.begin(), childContacts.end());
		}
		//Retrive all parents objects
		for (auto itr = m_Objects.begin(); itr != m_Objects.end(); itr++) {
			for (int i = 0; i < CHILD_TREE_NMB; i++) {
				for (auto childItr = nodes[i]->m_Objects.begin(); childItr != nodes[i]->m_Objects.end(); childItr++) {
					if ((*itr)->aabb.Contains((*childItr)->aabb)) {
						// contacts.push_back(p2Contact(*itr, *childItr)); //TO DO
					}
				}
			}
		}
	}

	return contacts;
}
