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
#define _USE_MATH_DEFINES
#include <cmath>
#include <p2physics.h>
#include <iostream>

int main()
{
	std::cout << "================= Matrix22 ===================\n";
	p2Mat22 m1(p2Vec2(1.0f, 2.0f), p2Vec2(3.0f, 4.0f));
	p2Mat22 m2(p2Vec2(5.0f, 6.0f), p2Vec2(7.0f, 8.0f));
	p2Vec2 v(9.0f, 10.0f);

	std::cout << "m1 : \n";
	m1.Show();
	std::cout << "m2 : \n";
	m2.Show();

	std::cout << "m1 + m2 : \n";
	(m1 + m2).Show();
	std::cout << "m1 - m2 : \n";
	(m1 - m2).Show();

	std::cout << "m1 += m2 : \n";
	m1 += m2;
	m1.Show();
	std::cout << "m1 -= m2 : \n";
	m1 -= m2;
	m1.Show();

	std::cout << "m1 * v : \n";
	(m1 * v).Show();

	std::cout << "\nm1 * m2 : \n";
	(m1 * m2).Show();

	std::cout << "m1 *= m2 : \n";
	m1 *= m2;
	m1.Show();

	std::cout << "m1 inverse : \n";
	m1.Invert().Show();

	std::cout << "m1 * m1 inverse : \n";
	(m1 * m1.Invert()).Show();

	std::cout << "================= Matrix33 ===================\n";
	p2Mat33 m3(p2Vec3(1.0f, 2.0f, 3.0f), p2Vec3(4.0f, 5.0f, 6.0f), p2Vec3(7.0f, 8.0f, 9.0f));
	p2Mat33 m4(p2Vec3(10.0f, 11.0f, 12.0f), p2Vec3(13.0f, 14.0f, 15.0f), p2Vec3(16.0f, 17.0f, 18.0f));
	p2Mat33 m5(p2Vec3(2.0f, -2.0f, 4.0f), p2Vec3(2.0f, 8.0f, -6.0f), p2Vec3(2.0f, 2.0f, -10.0f));
	p2Vec3 v1(19.0f, 20.0f, 21.0f);

	std::cout << "m3 : \n";
	m3.Show();
	std::cout << "m4 : \n";
	m4.Show();

	std::cout << "m3 + m4 : \n";
	(m3 + m4).Show();
	std::cout << "m3 - m4 : \n";
	(m3 - m4).Show();

	std::cout << "m3 += m4 : \n";
	m3 += m4;
	m3.Show();
	std::cout << "m3 -= m4 : \n";
	m3 -= m4;
	m3.Show();

	std::cout << "m3 * v1 : \n";
	(m3 * v1).Show();

	std::cout << "\nm3 * m4 : \n";
	(m3 * m4).Show();

	std::cout << "m3 *= m4 : \n";
	m3 *= m4;
	m3.Show();

	std::cout << "m5 : \n";
	m5.Show();

	std::cout << "m5 transposed : \n";
	m5.Transposed().Show();

	std::cout << "m5 inverse : \n";
	m5.Invert().Show();

	std::cout << "m5 * m5 inverse : \n";
	(m5 * m5.Invert()).Show();

	std::cout << "rotation 60° autour de l'axe x : \n";
	p2Mat33::RotationMatrix(60 * M_PI / 180.0, p2Vec3(1, 0, 0)).Show();
	std::cout << "rotation 60° autour de l'axe y : \n";
	p2Mat33::RotationMatrix(60 * M_PI / 180.0, p2Vec3(0, 1, 0)).Show();
	std::cout << "rotation 60° autour de l'axe z : \n";
	p2Mat33::RotationMatrix(60 * M_PI / 180.0, p2Vec3(0, 0, 1)).Show();
#if WIN32
	system("pause");
#endif
	return EXIT_SUCCESS;
}
