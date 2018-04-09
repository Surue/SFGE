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

p2ColliderDef::ShapeType p2Collider::GetShapeType()
{
	return shapeType;
}
