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


#ifndef SFGE_GAMEOBJECT_H
#define SFGE_GAMEOBJECT_H

#include <utility/json_utility.h>
#include <engine/engine.h>
#include <utility/python_utility.h>
//External includes
#include <SFML/System.hpp>
#include <pybind11/functional.h>
//STL includes
#include <list>
#include <string>

namespace sfge
{
class Component;
class Transform;
class Collider;
enum class ComponentType;

/**
* \brief The basic Game Object handler containing a list of Components
* it always contains a Transform component
*/
class GameObject
{
public:
	GameObject();
	~GameObject();
	void Init();
	/**
	* \brief Update the GameObject and its Components
	* \param dt Delta time since last frame
	*/
	void Update(sf::Time dt);
	/**
	* \brief Load a GameObject and create all its Component
	* \param gameObjectJson the sub json associated with the Game Object
	* \return the heap GameObject that will need to be destroyed
	*/
	static GameObject* LoadGameObject(Engine& engine, json& gameObjectJson);

	Component* AddComponent(Engine& engine, json& gameObjectJson);
	/**
	 * \brief Getter of the Transform attached to the GameObject
	 * \return Pointer to Transform
	 */
	Transform* GetTransform();
	/**
	* \breif Setter of the Transform attached to the GameObject
	*/
	void SetTransform(Transform* transform);

	/**
	 * \brief Get The Component of type given the T by template
	 * \return Return the first Component of type T that is attached to the GameObject
	 */
	template <class T>
	T* GetComponent()
	{
		for (auto component : m_Components)
		{
			auto castComponent = dynamic_cast<T*>(component);
			if (castComponent != nullptr)
			{
				return castComponent;
			}
		}
		return nullptr;
	};
	/**
	* \brief Get The Components of type given the T by template
	* \return Return the list of Component of type T that are attached to the GameObject
	*/
	template <class T>
	std::list<T*> GetComponents()
	{
		std::list<T*> componentsList;
		for (auto component : m_Components)
		{
			auto castComponent = dynamic_cast<T*>(component);
			if (castComponent != nullptr)
			{
				componentsList.push_back(castComponent);
			}
		}
		return componentsList;
	};

	/**
	* \brief Returns the first Component of type componentType given as argument, used by the Python Interpreter
	*/
	py::object GetComponentFromType(ComponentType componentType);
	/**
	* \brief Returns the first PyComponent of type componentType given as argument, used by the Python Interpreter
	*/
	py::object GetPyComponentFromType(py::handle pycomponentType);
	/**
	* \brief Returns the list of Components of type componentType given as argument, used by the Python Interpreter
	*/
	py::object GetComponentsFromType(ComponentType componentType);
	/**
	* \brief Return the reference to all the Component in the GameObject
	*/
	std::list<Component*>& GetAllComponents();

	/**
	 * \brief Get the name of the GameObject in the Scene
	 * \return Return the const reference of the string name
	 */
	const std::string& GetName();
	/**
	* \brief Setter of the Component name
	*/
	void SetName(std::string name);
	/**
	* \brief Triggered when one of the two Collider of the entering contact is a sensor
	*/
	void OnTriggerEnter(Collider* collider);
	/**
	* \brief Triggered when both Collider of the entering contact are not a sensor
	*/
	void OnCollisionEnter(Collider* collider);
	/**
	* \brief Triggered when one of the two Collider of the exiting contact is a sensor
	*/
	void OnTriggerExit(Collider* collider);
	/**
	* \brief Triggered when both Collider of the exiting contact are not a sensor
	*/
	void OnCollisionExit(Collider* collider);

protected:
	friend class Component;
	std::list<Component*> m_Components;
	std::string m_Name = "";
	Transform* m_Transform = nullptr;
};
}

template<> sfge::Component* sfge::GameObject::GetComponent<sfge::Component>();
template<> std::list<sfge::Component*> sfge::GameObject::GetComponents<sfge::Component>();
#endif
