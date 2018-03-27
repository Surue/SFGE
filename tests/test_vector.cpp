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
#include <p2physics.h>
#include <iostream>
int main()
{
	p2Vec2 vec1(3, 1);
	p2Vec2 vec2(5, 3);

	vec1.Show();
	vec2.Show();
	
	std::cout << "Addition: vec1 + vec2 = ";
	(vec1 + vec2).Show();

	vec1 += vec2;
	std::cout << "Vec1 apres += ";
	vec1.Show();
	std::cout << "Vec2 apres += ";
	vec2.Show();

	std::cout << "Soustraction: vec1 - vec2 = ";
	(vec1 - vec2).Show();
	vec1 -= vec2;
	std::cout << "Vec1 apres -= ";
	vec1.Show();
	std::cout << "Vec2 apres -= ";
	vec2.Show();

	vec1 = vec1 / 5;
	std::cout << "Vec1 / 5 = ";
	vec1.Show();

	vec1 = vec1 * 5;
	std::cout << "Vec1 * 5 = ";
	vec1.Show();

	std::cout << "Dot de vec1 et vec2 = " << p2Vec2::Dot(vec1, vec2) << "\n";

	std::cout << "Cross vec1 et vec2 = ";
	p2Vec2::Cross(vec1, vec2).Show();

	std::cout << "Vec1 magnitude = " << vec1.GetMagnitude() << "\n";

	std::cout << "Vec1 normalise = ";
	vec1.Normalized().Show();
	std::cout << "Vec1 = ";
	vec1.Show();
	vec1.Normalize();
	std::cout << "Vec1 apres normalisation = ";
	vec1.Show();

	p2Vec3 a(1.0f, 2.0f, 3.0f);
	p2Vec3 b(4.0f, 5.0f, 6.0f);

	(a + b).Show();
	(a - b).Show();

	std::cout << "Dot product: " << p2Vec3::Dot(a, b) << "\n";

	p2Vec3::Cross(a, b).Show();

	p2Vec3::Lerp(a, b, 0.5f).Show();

	p2Vec3::Proj(a, b).Show();
	p2Vec3::Proj(b, a).Show();

	p2Vec3::Refl(a, b).Show();
	p2Vec3::Refl(b, a).Show();

	std::cout << "Angle between the vectors: " << p2Vec3::AnglesBetween(a, b) << "\n";

#if WIN32
	system("pause");
#endif
	return EXIT_SUCCESS;
}
