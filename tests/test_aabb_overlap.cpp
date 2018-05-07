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

int main()
{
	std::ofstream file;
	file.exceptions(std::ios::failbit);
	try {
		file.open("data/scenes/test_aabb_overlap.scene", std::ios::in);
	}
	catch (std::ios_base::failure &fail) {
		std::cout << "ERROR: File not found\n";
		system("pause");
		return EXIT_FAILURE;
	}
	file.clear();
	file.exceptions(std::ios::goodbit);

	int columns = 20, row = 15;

	srand(time(NULL));

	if (file.is_open()) {
		file << "{\n\t\"name\": \"AABBOverlapTest\",\n\t\"game_objects\": [";
		for (int i = 0; i < columns; i++) {
			for (int j = 0; j < row; j++) {
				file << "\n\t{\n\t\t\"name\": \"Body" << i*columns + j<< "\",\n\t\t\"components\" : [ \n\t\t{";
				file << "\n\t\t\t\"type\" : 1, \n\t\t\t\"position\": [" << i * 50 << ", " << j * 50  << "],";
				file << "\n\t\t\t\"scale\": [1.0, 1.0], \n\t\t\t\"angle\": 0.0 \n\t\t},\n\t\t{";
				file << "\n\t\t\t\"name\": \"Rigidbody\",\n\t\t\t\"type\" : 5, \n\t\t\t\"body_type\": 2, \n\t\t\t\"gravity_scale\": 0 \n\t\t},\n\t\t{";
				float radius = 5;
				file << "\n\t\t\t\"name\": \"BoxCollider\",\n\t\t\t\"type\" : 6,\n\t\t\t\"collider_type\" : 1, \n\t\t\t\"radius\" : " << radius << ",\n\t\t\t\"sensor\" : false \n\t\t},\n\t\t{";
				file << "\n\t\t\t\"name\": \"Shape\",\n\t\t\t\"type\" : 3,\n\t\t\t\"shape_type\" : 1, \n\t\t\t\"radius\" : " << radius << " \n\t\t},\n\t\t{";
				file << "\n\t\t}\n\t\t]\n\t},";
			}
		}
		file << "\n\t{\n\t\t\"name\": \"Body\",\n\t\t\"components\" : [ \n\t\t{";
		file << "\n\t\t\t\"type\" : 1, \n\t\t\t\"position\": [" << 550 << ", " << 350 << "],";
		file << "\n\t\t\t\"scale\": [1.0, 1.0], \n\t\t\t\"angle\": 0.0 \n\t\t},\n\t\t{";
		file << "\n\t\t\t\"name\": \"Rigidbody\",\n\t\t\t\"type\" : 5, \n\t\t\t\"body_type\": 2, \n\t\t\t\"gravity_scale\": 0 \n\t\t},\n\t\t{";
		float radius = 15;
		file << "\n\t\t\t\"name\": \"BoxCollider\",\n\t\t\t\"type\" : 6,\n\t\t\t\"collider_type\" : 1, \n\t\t\t\"radius\" : " << radius << ",\n\t\t\t\"sensor\" : true \n\t\t},\n\t\t{";
		file << "\n\t\t\t\"name\": \"Shape\",\n\t\t\t\"type\" : 3,\n\t\t\t\"shape_type\" : 1, \n\t\t\t\"radius\" : " << radius << " \n\t\t},\n\t\t{";
		file << "\n\t\t\t\"type\" : 4,\n\t\t\t\"script_path\" : \"scripts/add_explosive_force_on_click.py\" \n\t\t},\n\t\t{";
		file << "\n\t\t}\n\t\t]\n\t},";

		file << "\n\t\t{\"name\": \"ScriptDebugDraw\",\n\t\t\"components\" : [\n\t\t{\n\t\t\t\"type\": 4,\n\t\t\t\"script_path\" : \"scripts/debug_draw_data.py\"}]}";
		file << "\n\t]\n}";

		file.close();
	}

	sfge::Engine engine;
	engine.Init(false, true);

	engine.GetSceneManager()->SetCurrentScene("data/scenes/test_aabb_overlap.scene");

	engine.Start();
#if WIN32
	system("pause");
#endif
	return EXIT_SUCCESS;
}