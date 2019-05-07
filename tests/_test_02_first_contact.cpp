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

#include <engine/engine.h>
#include <engine/scene.h>

#include <iostream>
#include <fstream>
#include <ctime>

int main()
{
	std::ofstream file;
	file.exceptions(std::ios::failbit);
	try {
		file.open("data/scenes/_test_2_first_contact.scene", std::ios::in);
	}
	catch (std::ios_base::failure &fail) {
		std::cout << "ERROR: File not found\n";
		system("pause");
		return EXIT_FAILURE;
	}
	file.clear();
	file.exceptions(std::ios::goodbit);

	int nbObject = 2000;

	srand(time(NULL));

	if (file.is_open()) {
		file << "{\n\t\"name\": \"QuadTreeTestScene\",\n\t\"game_objects\": [";
		for (int i = 0; i < nbObject; i++) {
			file << "\n\t{\n\t\t\"name\": \"Body" << i << "\",\n\t\t\"components\" : [ \n\t\t{";
			file << "\n\t\t\t\"type\" : 1, \n\t\t\t\"position\": [" << rand() % 1280 << ", " << rand() % 720 << "],";
			file << "\n\t\t\t\"scale\": [1.0, 1.0], \n\t\t\t\"angle\": 0.0 \n\t\t},\n\t\t{";
			file << "\n\t\t\t\"name\": \"Rigidbody\",\n\t\t\t\"type\" : 5, \n\t\t\t\"body_type\": 2, \n\t\t\t\"gravity_scale\": 0 \n\t\t},\n\t\t{";
			float sizeX = (rand() % 10) + 1;
			float sizeY = (rand() % 10) + 1;
			file << "\n\t\t\t\"name\": \"BoxShape\",\n\t\t\t\"type\" : 3,\n\t\t\t\"shape_type\" : 2, \n\t\t\t\"size\" : [" << sizeX << "," << sizeY << "]\n\t\t},\n\t\t{";
			file << "\n\t\t\t\"name\": \"BoxCollider\",\n\t\t\t\"type\" : 6,\n\t\t\t\"collider_type\" : 2, \n\t\t\t\"size\" : [" << sizeX << "," << sizeY << "],\n\t\t\t\"sensor\" : true \n\t\t},\n\t\t{";
			file << "\n\t\t\t\"type\": 4,\n\t\t\t\"script_path\" : \"scripts/box_random_movement.py\"\n\t\t},\n\t\t{";
			file << "\n\t\t\t\"type\": 4,\n\t\t\t\"script_path\" : \"scripts/contact_test.py\"";
			file << "\n\t\t}\n\t\t]\n\t},";
			
		}

		file << "\n\t\t{\"name\": \"ScriptDebugDraw\",\n\t\t\"components\" : [\n\t\t{\n\t\t\t\"type\": 4,\n\t\t\t\"script_path\" : \"scripts/debug_draw_data.py\"}]}";
		file << "\n\t]\n}";

		file.close();
	}

	sfge::Engine engine;
	engine.Init(false, true);

	engine.GetSceneManager()->SetCurrentScene("data/scenes/_test_2_first_contact.scene");

	uint32_t flags = 0x0020;

	engine.SetDebugDrawDataFlags(flags);

	engine.Start();

#if WIN32
	system("pause");
#endif
	return EXIT_SUCCESS;
}
