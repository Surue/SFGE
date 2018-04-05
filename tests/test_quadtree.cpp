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

#include <p2vector.h>
#include <p2aabb.h>
#include <p2body.h>
#include <p2world.h>
#include <p2shape.h>
#include <p2collider.h>

#include <SFML/Graphics.hpp>
#include <physics/physics.h>
#include <imgui-SFML.h>
#include <imgui.h>

#include <list>
#include <cstdio>      /* printf, scanf, puts, NULL */
#include <cstdlib>     /* srand, rand */
#include <ctime>       /* time */

#include <iostream>

int main()
{
	sf::RenderWindow window(sf::VideoMode(800, 800), "QuadTree test");
	//QuadTree quad(0, { b2Vec2(0,0), sfge::pixel2meter(sf::Vector2i(800,800)) });
	p2World world(p2Vec2(0.0f, 0.0f));

	srand(time(NULL));

	std::list<p2Body*> bodiesList;
	const int objNmb = 1000;
	for (int i = 0; i < objNmb; i++)
	{
		p2BodyDef bodyDef;
		bodyDef.position = sfge::pixel2meter(sf::Vector2f(rand() % 800, rand() % 800));
		bodyDef.linearVelocity = sfge::pixel2meter(sf::Vector2f(rand() % 10, rand() % 10));
		bodyDef.type = p2BodyType::DYNAMIC;
		p2Body* body = world.CreateBody(&bodyDef);

		p2ColliderDef colliderDef;
		
		p2RectShape rectShape;
		rectShape.SetSize(sfge::pixel2meter(sf::Vector2f((rand() % 100) - 50, (rand() % 100) - 50)));
		colliderDef.shape = &rectShape;

		body->CreateCollider(&colliderDef);

		bodiesList.push_back(body);
	}
	ImGui::SFML::Init(window);
	sf::Clock clock;
	while (window.isOpen())
	{
		sf::Time dt = clock.restart();
		// check all the window's events that were triggered since the last iteration of the loop
		sf::Event event;
		while (window.pollEvent(event))
		{
			ImGui::SFML::ProcessEvent(event);
			// "close requested" event: we close the window
			if (event.type == sf::Event::Closed)
				window.close();
		}
		ImGui::SFML::Update(window, dt);

		ImGui::Begin("Stats");
		{
			std::ostringstream oss;
			oss << "FPS: " << 1.0f / dt.asSeconds();

			ImGui::Text(oss.str().c_str());
		}

		ImGui::End();

		world.Step(dt.asSeconds());
		for (auto& body : bodiesList)
		{
			body->Step(dt.asSeconds());
			
			sf::Vector2f pos = sfge::meter2pixel(body->GetPosition());
			p2Vec2 v = body->GetLinearVelocity();
			if (pos.x < 0.0f && v.x<0.0f)
			{
				body->SetLinearVelocity(p2Vec2(-v.x, v.y));
			}
			else if (pos.x > 800.0f && v.x > 0.0f)
			{
				body->SetLinearVelocity(p2Vec2(-v.x, v.y));
			}
			if (pos.y < 0.0f && v.y<0.0f)
			{
				body->SetLinearVelocity(p2Vec2(v.x, -v.y));
			}
			else if (pos.y > 800.0f && v.y > 0.0f)
			{
				body->SetLinearVelocity(p2Vec2(v.x, -v.y));
			}
		}

		window.clear(sf::Color::Black);

		for (auto& obj : bodiesList)
		{
			//obj->Draw(window);

			for each (p2Shape shape in obj->GetShape())
			{
				//static_cast<p2RectShape&>(shape).GetSize().Show();
				sf::RectangleShape rectangle;
				rectangle.setPosition(sfge::meter2pixel(obj->GetPosition()) - sfge::meter2pixel(p2Vec2(0.01, 0.01)));
				rectangle.setSize(sfge::meter2pixel(p2Vec2(0.01, 0.01))*2.0f);

				window.draw(rectangle);
			}
		}

		//quad.Update(dt.asSeconds());
		//quad.Draw(window);
		ImGui::SFML::Render(window);

		window.display();
	}
	ImGui::SFML::Shutdown();

	return EXIT_SUCCESS;
}