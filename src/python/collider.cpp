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


#include <physics/collider.h>
#include <physics/body2d.h>
#include <physics/physics.h>

#include <graphics/shape.h>

namespace sfge
{
void Collider::Init()
{
}
void Collider::Update(float dt)
{
}
void Collider::OnColliderEnter(Collider * collider)
{
	if (collider->m_PhysicsCollider->IsSensor() or m_PhysicsCollider->IsSensor())
	{
		m_GameObject->OnTriggerEnter(collider);
	}
	else
	{
		m_GameObject->OnCollisionEnter(collider);
	}
}

void Collider::OnColliderExit(Collider * collider)
{
	if (collider->m_PhysicsCollider->IsSensor() or m_PhysicsCollider->IsSensor())
	{
		m_GameObject->OnTriggerExit(collider);
	}
	else
	{
		m_GameObject->OnCollisionExit(collider);
	}
}

Collider* Collider::LoadCollider(Engine & engine, GameObject * gameObject, json & componentJson)
{
	Body2d* body2d = gameObject->GetComponent<Body2d>();
	if (body2d != nullptr)
	{
		Collider* collider = new Collider(gameObject);
		p2ColliderDef colliderDef;
		colliderDef.userData = collider;
		
		ColliderType colliderType = ColliderType::NONE;
		if (CheckJsonNumber(componentJson, "collider_type"))
		{
			colliderType = (ColliderType)componentJson["collider_type"];
		}
		p2Shape* shape = nullptr;
		switch (colliderType)
		{
		case ColliderType::CIRCLE:
			{
				p2CircleShape* circleShape = new p2CircleShape();
				if (CheckJsonNumber(componentJson, "radius"))
				{
					circleShape->SetRadius(pixel2meter((float)componentJson["radius"]));
				}
				shape = circleShape;
				colliderDef.shapeType = p2ColliderDef::ShapeType::CIRCLE;
			}
		break;
		case ColliderType::RECTANGLE:
			{
				sf::Vector2f boxSize = GetVectorFromJson(componentJson, "size");
				p2PolygonShape* polyShape = new p2PolygonShape();
				polyShape->SetVerticesCount(4);
				
				polyShape->SetVertice(pixel2meter(sf::Vector2f(-boxSize.x * 0.5f, boxSize.y * 0.5f)), 0);
				polyShape->SetVertice(pixel2meter(sf::Vector2f(-boxSize.x * 0.5f, -boxSize.y * 0.5f)), 1);
				polyShape->SetVertice(pixel2meter(sf::Vector2f(boxSize.x * 0.5f, -boxSize.y * 0.5f)), 2);
				polyShape->SetVertice(pixel2meter(sf::Vector2f(boxSize.x * 0.5f, boxSize.y * 0.5f)), 3);

				shape = polyShape;
				colliderDef.shapeType = p2ColliderDef::ShapeType::POLYGON;
				break; 
			}
		case ColliderType::POLYGON: //TO DO AJOUTER VERIFICATION CONVEXE, SINON TRANSFORMER EN CONVEXE
			{
				p2PolygonShape* polygonShape = new p2PolygonShape();
				int verticesCount = (int)componentJson["point_count"];
				polygonShape->SetVerticesCount(verticesCount);
				for (int i = 0; i < verticesCount; i++)
				{
					polygonShape->SetVertice(pixel2meter(GetVectorFromJson(componentJson, "point" + std::to_string(i))), i );
				}

				shape = polygonShape;
				colliderDef.shapeType = p2ColliderDef::ShapeType::POLYGON;
			}
		}
		if (CheckJsonExists(componentJson, "offset"))
		{
			colliderDef.offset = pixel2meter(GetVectorFromJson(componentJson, "offset"));
		}
		if (CheckJsonNumber(componentJson, "bouncing"))
		{
			colliderDef.restitution = componentJson["bouncing"];
		}
		if (CheckJsonNumber(componentJson, "friction"))
		{
			colliderDef.friction = componentJson["friction"];
		}
		if (shape != nullptr)
		{
			colliderDef.shape = shape;
		}
		if (CheckJsonParameter(componentJson, "sensor", json::value_t::boolean))
		{
			colliderDef.isSensor = componentJson["sensor"];
		}

		collider->m_PhysicsCollider = body2d->GetBody()->CreateCollider(&colliderDef);
		
		if (shape != nullptr)
		{
			delete(shape);
		}

		return collider;
	}
	else
	{
		Log::GetInstance()->Error("No body attached on the GameObject");
	}
	return nullptr;
}

}
