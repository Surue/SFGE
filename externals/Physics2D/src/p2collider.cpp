#include "..\include\p2collider.h"
#include <p2body.h>

#include <iostream>

#include <p2matrix.h>

#define _CRTDBG_MAP_ALLOC
#include<iostream>
#include <crtdbg.h>
#ifdef _DEBUG
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

p2Collider::p2Collider()
{
}

p2Collider::p2Collider(p2ColliderDef colliderDef, p2Body* body)
{
	userData = colliderDef.userData;
	isSensor = colliderDef.isSensor;
	restitution = colliderDef.restitution;
	shape = colliderDef.shape->Clone();
	shapeType = colliderDef.shapeType;

	m_Body = body;

	m_Offset = p2Vec2(colliderDef.offset.x, colliderDef.offset.y);
	float radius = 0;
	p2Vec2 size;
}

p2Collider::~p2Collider()
{
	delete(shape);
}

bool p2Collider::IsSensor() const
{
	return isSensor;
}

void * p2Collider::GetUserData() const
{
	return userData;
}

p2Shape* p2Collider::GetShape() const
{	
	return shape;
}

p2Vec2& p2Collider::GetOffset()
{
	return m_Offset;
}

p2Vec2 p2Collider::GetPosition() const
{
	return (p2Mat22::RotationMatrix(m_Body->GetAngle()) * m_Offset) + m_Body->GetPosition();
}

p2ColliderDef::ShapeType p2Collider::GetShapeType() const
{
	return shapeType;
}
