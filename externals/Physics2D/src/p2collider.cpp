#include "..\include\p2collider.h"

#include <iostream>

p2Collider::p2Collider()
{
}

p2Collider::p2Collider(p2ColliderDef colliderDef)
{
	userData = colliderDef.userData;
	isSensor = colliderDef.isSensor;
	restitution = colliderDef.restitution;
	shape = colliderDef.shape;
	shapeType = colliderDef.shapeType;

	switch (colliderDef.shapeType) {
		case p2ColliderDef::ShapeType::CIRCLE:
			shape = new p2CircleShape();
			static_cast<p2CircleShape*>(shape)->SetRadius(static_cast<p2CircleShape*>(colliderDef.shape)->GetRadius());
			break;

		case p2ColliderDef::ShapeType::RECT:
			shape = new p2RectShape();
			static_cast<p2RectShape*>(shape)->SetSize(static_cast<p2RectShape*>(colliderDef.shape)->GetSize());
			break;
	}
	m_Offset = p2Vec2(colliderDef.offset.x, colliderDef.offset.y);
}

p2Collider::~p2Collider()
{
}

bool p2Collider::IsSensor()
{
	return isSensor;
}

void * p2Collider::GetUserData()
{
	return nullptr;
}

p2Shape* p2Collider::GetShape()
{	
	return shape;
}

p2Vec2 p2Collider::GetOffset()
{
	return m_Offset;
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
