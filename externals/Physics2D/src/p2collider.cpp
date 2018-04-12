#include "..\include\p2collider.h"
#include <p2body.h>

#include <iostream>

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

bool p2Collider::IsSensor()
{
	return isSensor;
}

void * p2Collider::GetUserData()
{
	return userData;
}

void p2Collider::Step(float dt)
{
	shape->ComputeAABB(&aabb, GetPosition(), m_Body->GetAngle());
}

p2Shape* p2Collider::GetShape()
{	
	return shape;
}

p2Vec2 p2Collider::GetOffset()
{
	return m_Offset;
}

p2Vec2 p2Collider::GetPosition()
{
	return p2Vec2(m_Body->GetPosition() + m_Offset);
}

std::string p2Collider::GetShapeJson()
{
	std::string jsonString = "{";

	switch (GetShapeType()) {
		case p2ColliderDef::ShapeType::CIRCLE:
			jsonString += static_cast<p2CircleShape*>(GetShape())->GetJson();
			break;

		case p2ColliderDef::ShapeType::RECT:
			jsonString += static_cast<p2RectShape*>(GetShape())->GetJson();
			break;
	}

	jsonString += ", \"offset\" : [" + std::to_string(m_Offset.x * 100) + "," + std::to_string(m_Offset.y * 100) + "]";

	jsonString += "}";

	return jsonString;
}

p2ColliderDef::ShapeType p2Collider::GetShapeType()
{
	return shapeType;
}
