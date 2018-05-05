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

#ifndef SFGE_INPUT_H
#define SFGE_INPUT_H

#include <engine/engine.h>
#include <SFML/Window/Keyboard.hpp>
#include <SFML/Window/Mouse.hpp>
#include <SFML/System/Time.hpp>

namespace sfge
{
class KeyboardManager;
class MouseManager;
/**
* \brief Handles Input like the Keyboard, the Joystick or the Mouse
*/
class InputManager : public Module
{
public:
	using Module::Module;

	InputManager(Engine& engine, bool enable = true);
	/**
	 * \brief Initialize the Input Manager
	 */
	void Init() override;
	/**
	 * \brief Update called each frame to report input status
	 * \param dt Delta time since last frame
	 */
	void Update(sf::Time dt) override;
	/**
	* \brief Called at the end of the program
	*/
	void Destroy() override;
	/**
	* \brief Called before the loading of a new Scene
	*/
	void Reset() override;
	/**
	* \brief Called at the end of the loading frame
	*/
	void Collect() override;
	/**
	* \brief Getter of the KeyboardManager
	*/
	KeyboardManager* GetKeyboardManager();
	/**
	* \brief Getter of the MouseManager
	*/
	MouseManager* GetMouseManager();

private:

protected:
	KeyboardManager* m_KeyboardManager = nullptr;
	MouseManager* m_MouseManager = nullptr;
};

/**
* \brief Struct used to retireve the Keyboard Key status
*/
struct KeyPressedStatus { bool previousKeyPressed; bool keyPressed; };

/**
* \brief Manage the state of the Keyboard
*/
class KeyboardManager
{
public:
	/**
	* \brief Called each frame to update the keyboard status
	*/
	void Update(sf::Time dt);
	/**
	* \brief Get if the key is held 
	*/
	bool IsKeyHeld(sf::Keyboard::Key key);
	/**
	* \brief Get if the key is down
	*/
	bool IsKeyDown(sf::Keyboard::Key key);
	/**
	* \brief Get if the key is up 
	*/
	bool IsKeyUp(sf::Keyboard::Key key);

protected:
	

private:
	KeyPressedStatus keyPressedStatusArray[sf::Keyboard::Key::KeyCount] ={};
};

struct MouseKeyPressedStatus { bool previousMouseKeyPressed; bool mouseKeyPressed; };

/**
* \brief Manage the mouse position status
*/
class MouseManager
{
public:
	/**
	* \brief Called each frame to update the keyboard status
	*/
	void Update(sf::Time dt);

	bool IsButtonHeld(sf::Mouse::Button button);
	bool IsButtonDown(sf::Mouse::Button button);
	bool IsButtonUp(sf::Mouse::Button button);

	sf::Vector2i localPosition(std::shared_ptr<sf::RenderWindow> window);

private:
	MouseKeyPressedStatus mouseKeyPressesStatusArray[sf::Mouse::Button::ButtonCount] = {};
};

}
#endif
