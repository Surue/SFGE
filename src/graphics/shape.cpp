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

#include <graphics/graphics.h>
#include <graphics/shape.h>
#include <utility/json_utility.h>
#include <engine/log.h>
#include <engine/transform.h>

namespace sfge
{

Shape::Shape(GameObject* gameObject):
		Component(gameObject)
{
}

void Shape::Init()
{
}

void Shape::Update(float time)
{
	if (m_Shape != nullptr)
	{
		m_Shape->setRotation(m_GameObject->GetTransform()->GetEulerAngle());
		m_Shape->setPosition(m_GameObject->GetTransform()->GetPosition() + m_Offset);
	}
}

void Shape::Draw(sf::RenderWindow& window)
{
	if(m_Shape != nullptr)
	{
		window.draw(*m_Shape);
	}
}

void Shape::SetFillColor(sf::Color color)
{
	if (m_Shape != nullptr)
	{
		m_Shape->setFillColor(color);
	}
}

Shape* Shape::LoadShape(Engine& engine, json& componentJson, GameObject* gameObject)
{
	Shape* shape = nullptr;
	sf::Vector2f offset;
	

	if(CheckJsonNumber(componentJson, "shape_type"))
	{
		ShapeType shapeType = (ShapeType)componentJson["shape_type"];
		switch(shapeType)
		{
		case ShapeType::CIRCLE:
			shape = Circle::LoadCircle(componentJson, gameObject);
			break;
		case ShapeType::RECTANGLE:
			shape = Rectangle::LoadRectangle(componentJson, gameObject);
			break;
		case ShapeType::POLYGON:
			shape = Polygon::LoadPolygon(componentJson, gameObject);
			break;
		}
	}
	offset = GetVectorFromJson(componentJson, "offset");
	
	if(shape != nullptr)
	{
		{
			//Log::GetInstance()->Msg("Load shape");
		}
		shape->SetOffset(offset);
		auto graphicsManager = engine.GetGraphicsManager();
		graphicsManager->GetShapeManager()->AddShape(shape);
	}
	return shape;
}

Circle::Circle(GameObject* gameObject,   float radius):
		Shape(gameObject)
{
	m_Radius = radius;
	m_Shape = std::make_shared<sf::CircleShape>(radius);
	m_Shape->setFillColor(sf::Color::Green);
	m_Shape->setOrigin(sf::Vector2f(radius, radius));
}

void Circle::Update(float dt)
{
	if (m_Shape != nullptr)
	{
		m_Shape->setPosition(m_GameObject->GetTransform()->GetPosition() + m_Offset);
	}
}



Circle* Circle::LoadCircle(json& componentJson, GameObject* gameObject)
{
	float radius = 1.0f;

	if(CheckJsonNumber(componentJson, "radius"))
	{
		radius = componentJson["radius"];
	}

	return new Circle(gameObject, radius);

}
Rectangle::Rectangle(GameObject* gameObject, sf::Vector2f size):
		Shape(gameObject)
{
	m_Size = size;
	m_Shape = std::make_shared<sf::RectangleShape>(size);
	m_Shape->setFillColor(sf::Color::Red);
	m_Shape->setOrigin(size / 2.0f);
}

void Rectangle::Update(float time)
{
	if (m_Shape != nullptr)
	{
		m_Shape->setRotation(m_GameObject->GetTransform()->GetEulerAngle());
		m_Shape->setPosition(m_GameObject->GetTransform()->GetPosition() + m_Offset);
	}
}

Rectangle* Rectangle::LoadRectangle(json& componentJson, GameObject* gameObject)
{

	sf::Vector2f size;
	size = GetVectorFromJson(componentJson, "size");
	{
		std::ostringstream oss;
		oss << "Loading Rectangle with size: " << size.x << ", " << size.y;
		//Log::GetInstance()->Msg(oss.str());
	}
	return new Rectangle(gameObject, size);
}


ShapeManager::ShapeManager(GraphicsManager& graphicsManager):
		m_GraphicsManager(graphicsManager)
{

}


void ShapeManager::Draw(sf::RenderWindow& window)
{
	for(auto shape : m_Shapes)
	{
		shape->Draw(window);
	}
}

void ShapeManager::Reset()
{
	for (auto shape : m_Shapes)
	{
		delete(shape);
	}
	m_Shapes.clear();
}

void ShapeManager::Reload()
{
}

void ShapeManager::AddShape(Shape* shape)
{
	m_Shapes.push_back(shape);
}

Polygon::Polygon(GameObject * gameObject, int pointCount, std::vector<sf::Vector2f> points): Shape(gameObject)
{
	m_PointCount = pointCount;
	int i = 0;

	m_Points.resize(m_PointCount);

	sf::ConvexShape tmpShape(m_PointCount);
	
	for (int i = 0; i < points.size(); i++) {
		m_Points[i] = points[i];
		tmpShape.setPoint(i, points[i]);
	}

	m_Shape = std::make_shared<sf::ConvexShape>(tmpShape);

	m_Shape->setFillColor(sf::Color::Green);
	m_Shape->setOrigin(sf::Vector2f(0, 0));
}

void Polygon::Update(float time)
{
	if (m_Shape != nullptr)
	{
		m_Shape->setRotation(m_GameObject->GetTransform()->GetEulerAngle());
		m_Shape->setPosition(m_GameObject->GetTransform()->GetPosition() + m_Offset);
	}
}

Polygon * Polygon::LoadPolygon(json & componentJson, GameObject * gameObject)
{
	int pointCount = (int)componentJson["point_count"];
	std::vector<sf::Vector2f> points;
	points.resize(pointCount);

	for (int i = 0; i < pointCount; i++)
	{
		points[i] = GetVectorFromJson(componentJson, "point" + std::to_string(i));
	}

	{
		std::ostringstream oss;
		oss << "Loading Polygon with " << pointCount << " points:\n";
		for (sf::Vector2f point : points) {
			oss << "[" << point.x << ", " << point.y << "]";
		}
		//Log::GetInstance()->Msg(oss.str());
	}

	return new Polygon(gameObject, pointCount, points);
}

}
