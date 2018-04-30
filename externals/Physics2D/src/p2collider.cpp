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
		size = static_cast<p2RectShape*>(shape)->GetSize();
		m_Inertia = (1.0f / 12.0f) * m_Body->GetMass() * (size.x * size.x + size.y * size.y);
		m_InvInertia = 1.0f / m_Inertia;
		m_Centroide = m_Offset;
		break;
	case p2ColliderDef::ShapeType::CIRCLE:
		m_Inertia = 0.5f * m_Body->GetMass() * static_cast<p2CircleShape*>(shape)->GetRadius() * static_cast<p2CircleShape*>(shape)->GetRadius();
		m_InvInertia = 1.0f / m_Inertia;
		m_Centroide = m_Offset;
		break;

	case p2ColliderDef::ShapeType::POLYGON:
		//Find the inertia
		m_Inertia =  (1.0f / 6.0f) * m_Body->GetMass();

		float inertia = 0;
		float inertiaDivisor = 0;
		std::vector<p2Vec2> tmpVertices = static_cast<p2PolygonShape*>(shape)->GetVertices();
		for (int i = 0; i < tmpVertices.size(); ++i) {
			p2Vec2 p1 = tmpVertices[i];
			p2Vec2 p2;
			if (i + 1 < tmpVertices.size()) {
				p2 = tmpVertices[i + 1];
			}
			else {
				p2 = tmpVertices[0];
			}

			inertia += abs(p2Vec2::Cross(p2, p1).z) * (p2Vec2::Dot(p1, p1) + p2Vec2::Dot(p1, p2) + p2Vec2::Dot(p2, p2));
			inertiaDivisor += abs(p2Vec2::Cross(p2, p1).z);
		}

		m_Inertia *= (inertia / inertiaDivisor);

		m_InvInertia = 1.0f / m_Inertia;
		//Find the centroide
		p2Vec2 centroid = p2Vec2(0, 0);
		float area = 0.0f;

		p2Vec2 positionRef = p2Vec2(0, 0);

		for (int i = 0; i < tmpVertices.size(); ++i) {
			positionRef += tmpVertices[i];
		}

		positionRef *= 1.0f / tmpVertices.size();

		const float inv = 1.0f / 3.0f;

		for (int i = 0; i < tmpVertices.size(); ++i) {
			p2Vec2 p1 = positionRef;
			p2Vec2 p2 = tmpVertices[i];
			p2Vec2 p3;
			if (i + 1 < tmpVertices.size()) {
				p3 = tmpVertices[i + 1];
			}
			else {
				p3 = tmpVertices[0];
			}

			p2Vec2 edge1 = p2 - p1;
			p2Vec2 edge2 = p3 - p1;

			float crossValue = p2Vec2::Cross(edge1, edge2).z;

			float areaTriangle = 0.5f * crossValue;
			area += areaTriangle;

			centroid += (p1 + p2 + p3) * inv * areaTriangle;
		}

		centroid *= 1.0f / area;
		m_Centroide = centroid + m_Offset + positionRef;
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

float p2Collider::GetInertia()
{
	return m_Inertia;
}
