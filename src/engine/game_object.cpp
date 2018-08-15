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

#include <engine/game_object.h>
#include <engine/component.h>
#include <engine/transform.h>
#include <engine/log.h>
#include <python/python_engine.h>
#include <pybind11/stl.h>
#include <pybind11/pytypes.h>
#include <memory>

#include <graphics/sprite.h>
#include <python/pycomponent.h>
#include <graphics/shape.h>
#include <physics/body2d.h>
#include <physics/collider.h>
#include <audio/audio.h>

namespace sfge
{
void GameObject::Update(sf::Time dt)
{
	for(auto component : m_Components)
	{
		if(component == nullptr)
		{
			Log::GetInstance()->Error("Component in GameObject is null");
			continue;
		}
		component->Update(dt.asSeconds());
	}
}

GameObject* GameObject::LoadGameObject(Engine& engine, json& gameObjectJson)
{
	GameObject* gameObject = new GameObject();
	if (CheckJsonParameter(gameObjectJson, "name", json::value_t::string))
	{
		gameObject->m_Name = gameObjectJson["name"].get<std::string>();
	}
	else
	{
		gameObject->m_Name = "GameObject";
	}

	{
		std::ostringstream oss;
		oss << "Loading game_object: " << gameObject->m_Name;
		//Log::GetInstance()->Msg(oss.str());
	}

	if (CheckJsonParameter(gameObjectJson, "components", json::value_t::array))
	{
		for (json& componentJson : gameObjectJson["components"])
		{
			auto newComponent = Component::LoadComponent(engine, componentJson, gameObject);
			if(newComponent != nullptr)
			{
				gameObject->m_Components.push_back(newComponent);
			}
			else
			{
				std::ostringstream oss;
				oss << "Component from: " << componentJson << " is nullptr";
				Log::GetInstance()->Error(oss.str());
			}
		}
	}
	else
	{
		Log::GetInstance()->Error("There are no Component attached to the GameObject");
	}
	gameObject->Init();
	//if there is no transform, it is always at the beginning of the list
	if(gameObject->m_Transform == nullptr)
	{
		auto transform = new Transform(gameObject);
		transform->SetName("Transform");
		gameObject->m_Components.push_front(transform);
		gameObject->SetTransform(transform);
	}
	return gameObject;
}

Component * GameObject::AddComponent(Engine & engine, json & gameObjectJson)
{
	auto newComponent = Component::LoadComponent(engine, gameObjectJson, this);
	if (newComponent != nullptr)
	{
		m_Components.push_back(newComponent);
	}

	return newComponent;
}

Transform* GameObject::GetTransform()
{
	return m_Transform;
}

void GameObject::SetTransform(Transform* transform)
{
	m_Transform = transform;
}

GameObject::GameObject()
{
}

GameObject::~GameObject()
{
	{
		Log::GetInstance()->Error("DESTROY GAME OBJECT " + m_Name);
	}
	m_Components.clear();
	delete(m_Transform);
	m_Transform = nullptr;
	
}

void GameObject::Init()
{
	for (auto& comp : m_Components)
	{
		comp->Init();
	}
}

py::object GameObject::GetComponentFromType(ComponentType componentType)
{
	switch (componentType)
	{
	case ComponentType::SPRITE:
		return py::cast(GetComponent<Sprite>());
	case ComponentType::PYCOMPONENT:
		return py::cast(GetComponent<PyComponent>());
	case ComponentType::SHAPE:
		return py::cast(GetComponent<Shape>());
	case ComponentType::BODY2D:
		return py::cast(GetComponent<Body2d>());
	case ComponentType::COLLIDER:
		return py::cast(GetComponent<Collider>());
	case ComponentType::SOUND:
		return py::cast(GetComponent<Sound>());
	}
	return py::none();
}

py::object GameObject::GetPyComponentFromType(py::handle pycomponentType)
{
	for (PyComponent* pyComponent : GetComponents<PyComponent>())
	{
		if (py::cast(pyComponent).get_type().is(pycomponentType))
		{
			return py::cast(pyComponent);
		}
	}
	return py::none();
}

py::object GameObject::GetComponentsFromType(ComponentType componentType)
{
	switch (componentType)
	{
	case ComponentType::SPRITE:
		return py::cast(GetComponents<Sprite>());
	case ComponentType::PYCOMPONENT:
		return py::cast(GetComponents<PyComponent>());
	case ComponentType::SHAPE:
		return py::cast(GetComponents<Shape>());
	case ComponentType::BODY2D:
		return py::cast(GetComponents<Body2d>());
	case ComponentType::COLLIDER:
		return py::cast(GetComponents<Collider>());
	case ComponentType::SOUND:
		return py::cast(GetComponents<Sound>());
	}
	return py::none();
}

std::list<Component*>& GameObject::GetAllComponents()
{
	return m_Components;
}

const std::string & GameObject::GetName()
{
	return m_Name;
}

void GameObject::SetName(std::string name)
{
	m_Name = name;
}

void GameObject::OnTriggerEnter(Collider * collider)
{
	for (auto comp : m_Components)
	{
		comp->OnTriggerEnter(collider);
	}
}

void GameObject::OnCollisionEnter(Collider * collider)
{
	for (auto comp : m_Components)
	{
		comp->OnCollisionEnter(collider);
	}
}

void GameObject::OnTriggerExit(Collider * collider)
{
	for (auto comp : m_Components)
	{
		comp->OnTriggerExit(collider);
	}
}

void GameObject::OnCollisionExit(Collider * collider)
{
	for (auto comp : m_Components)
	{
		comp->OnCollisionExit(collider);
	}
}

}
