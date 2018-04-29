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

	switch (shapeType) {
	case p2ColliderDef::ShapeType::RECT:
	case p2ColliderDef::ShapeType::CIRCLE:
		m_Centroide = m_Offset;
		break;

	case p2ColliderDef::ShapeType::POLYGON:
		m_Centroide = p2Vec2(0, 0);

		float signedArea = 0.0f;
		float area = 0.0f;

		std::vector<p2Vec2> tmpVertices = static_cast<p2PolygonShape*>(shape)->GetVertices();
		int i = 0;
		for (i = 0; i < tmpVertices.size() - 1; ++i) {

			area = p2Vec2::Cross(tmpVertices[i], tmpVertices[i + 1]).z;
			signedArea += area;
			m_Centroide += (tmpVertices[i] + tmpVertices[i + 1]) * area;
		}

		area = p2Vec2::Cross(tmpVertices[i], tmpVertices[0]).z;
		signedArea += area;
		m_Centroide += (tmpVertices[i] + tmpVertices[0]) * area;

		signedArea *= 0.5f;

		m_Centroide = m_Centroide / (6.0f * signedArea) + m_Offset;
		break;
	}
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

p2Vec2 p2Collider::GetPosition()
{
	return (p2Mat22::RotationMatrix(m_Body->GetAngle()) * m_Offset) + m_Body->GetPosition();
}

p2ColliderDef::ShapeType p2Collider::GetShapeType() const
{
	return shapeType;
}

p2Body * p2Collider::GetBody() const
{
	return m_Body;
}

float p2Collider::GetRestitution() const
{
	return restitution;
}

p2Vec2 p2Collider::GetCentroide()
{
	return m_Centroide;
}
