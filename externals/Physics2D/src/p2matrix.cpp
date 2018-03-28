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

#include <p2matrix.h>
#include <iostream>
#include <cmath>

p2Mat22::p2Mat22()
{
}

p2Mat22::p2Mat22(p2Vec2 v1, p2Vec2 v2)
{
	this->v1 = v1;
	this->v2 = v2;
}

p2Mat22 p2Mat22::operator+(p2Mat22 m)
{
	return p2Mat22(v1 + m.v1, v2 + m.v2);
}

p2Mat22 p2Mat22::operator+=(p2Mat22 m)
{
	v1 += m.v1;
	v2 += m.v2;
	return p2Mat22(v1, v2);
}

p2Mat22 p2Mat22::operator-(p2Mat22 m)
{
	return p2Mat22(v1 - m.v1, v2 - m.v2);
}

p2Mat22 p2Mat22::operator-=(p2Mat22 m)
{
	v1 -= m.v1;
	v2 -= m.v2;
	return p2Mat22(v1, v2);
}

p2Vec2 p2Mat22::operator*(p2Vec2 v)
{
	return p2Vec2(v1.x * v.x + v2.x * v.y,
				  v1.y * v.x + v2.y * v.y);
}

p2Mat22 p2Mat22::operator*(p2Mat22 m)
{
	return p2Mat22(p2Vec2((v1.x * m.v1.x) + (v2.x * m.v1.y),
		  					 (v1.y * m.v1.x) + (v2.y * m.v1.y)), 
					  p2Vec2((v1.x * m.v2.x) + (v2.x * m.v2.y),
						     (v1.y * m.v2.x) + (v2.y * m.v2.y))
		              );
}

p2Mat22 p2Mat22::operator*=(p2Mat22 m)
{
	p2Vec2 a = p2Vec2((v1.x * m.v1.x) + (v2.x * m.v1.y),
					  (v1.y * m.v1.x) + (v2.y * m.v1.y));
	v2 = p2Vec2((v1.x * m.v2.x) + (v2.x * m.v2.y),
				(v1.y * m.v2.x) + (v2.y * m.v2.y));

	v1 = a;
	return p2Mat22(v1, v2);
}

p2Mat22 p2Mat22::RotationMatrix(float angle)
{
	return p2Mat22(p2Vec2(cos(angle), sin(angle)),
					  p2Vec2(-sin(angle), cos(angle)));
}

p2Mat22 p2Mat22::Invert()
{
	float det = 1 / ((v1.x * v2.y) - (v1.y * v2.x));
	std::cout << v1.x << " v1.x, " << v1.y << " v1.y, " << v2.x << " v2.x, " << v2.y << " v2.y \n";
	std::cout << "det = " << det << "\n";
	return p2Mat22(p2Vec2(v2.y * det, -v1.y * det), 
					  p2Vec2(-v2.x * det, v1.x * det));
}

void p2Mat22::Show()
{
	std::cout << "| " << v1.x << " " << v2.x << " | \n| " << v1.y << " " << v2.y << " |" << "\n \n";
}

p2Mat33::p2Mat33()
{
}

p2Mat33::p2Mat33(p2Vec3 v1, p2Vec3 v2, p2Vec3 v3)
{
	this->v1 = v1;
	this->v2 = v2;
	this->v3 = v3;
}

p2Mat33 p2Mat33::operator+(p2Mat33 m)
{
	return p2Mat33(v1 + m.v1, v2 + m.v2, v3 + m.v3);
}

p2Mat33 p2Mat33::operator+=(p2Mat33 m)
{
	v1 += m.v1;
	v2 += m.v2;
	v3 += m.v3;
	return p2Mat33(v1, v2, v3);
}

p2Mat33 p2Mat33::operator-(p2Mat33 m)
{
	return p2Mat33(v1 - m.v1, v2 - m.v2, v3 - m.v3);
}

p2Mat33 p2Mat33::operator-=(p2Mat33 m)
{
	v1 -= m.v1;
	v2 -= m.v2;
	v3 -= m.v3;
	return p2Mat33(v1, v2, v3);
}

p2Vec3 p2Mat33::operator*(p2Vec3 v)
{
	return p2Vec3((v1.x * v.x) + (v2.x * v.y) + (v3.x * v.z),
				  (v1.y * v.x) + (v2.y * v.y) + (v3.y * v.z),
			      (v1.z * v.x) + (v2.z * v.y) + (v3.z * v.z));
}

p2Mat33 p2Mat33::operator*(p2Mat33 m)
{
	return p2Mat33(p2Vec3(v1.x * m.v1.x + v2.x * m.v1.y + v3.x * m.v1.z,
						     v1.y * m.v1.x + v2.y * m.v1.y + v3.y * m.v1.z,
							 v1.z * m.v1.x + v2.z * m.v1.y + v3.z * m.v1.z),
				      p2Vec3(v1.x * m.v2.x + v2.x * m.v2.y + v3.x * m.v2.z,
							 v1.y * m.v2.x + v2.y * m.v2.y + v3.y * m.v2.z,
							 v1.z * m.v2.x + v2.z * m.v2.y + v3.z * m.v2.z),
					  p2Vec3(v1.x * m.v3.x + v2.x * m.v3.y + v3.x * m.v3.z,
							 v1.y * m.v3.x + v2.y * m.v3.y + v3.y * m.v3.z,
							 v1.z * m.v3.x + v2.z * m.v3.y + v3.z * m.v3.z)
	);
}

p2Mat33 p2Mat33::operator*=(p2Mat33 m)
{
	p2Vec3 a = p2Vec3(v1.x * m.v1.x + v2.x * m.v1.y + v3.x * m.v1.z,
		v1.y * m.v1.x + v2.y * m.v1.y + v3.y * m.v1.z,
		v1.z * m.v1.x + v2.z * m.v1.y + v3.z * m.v1.z);

	p2Vec3 b = p2Vec3(v1.x * m.v2.x + v2.x * m.v2.y + v3.x * m.v2.z,
		v1.y * m.v2.x + v2.y * m.v2.y + v3.y * m.v2.z,
		v1.z * m.v2.x + v2.z * m.v2.y + v3.z * m.v2.z);

	v3 = p2Vec3(v1.x * m.v3.x + v2.x * m.v3.y + v3.x * m.v3.z,
		v1.y * m.v3.x + v2.y * m.v3.y + v3.y * m.v3.z,
		v1.z * m.v3.x + v2.z * m.v3.y + v3.z * m.v3.z);

	v1 = a;
	v2 = b;

	return p2Mat33(v1, v2, v3);
}

p2Mat33 p2Mat33::operator*(float f)
{
	return p2Mat33(v1 * f, v2 * f, v3 * f);
}

p2Mat33 p2Mat33::operator+=(float f)
{
	v1 = v1 * f;
	v2 = v2 * f;
	v3 = v3 * f;
	return p2Mat33(v1, v2, v3);
}

p2Mat33 p2Mat33::RotationMatrix(float angle, p2Vec3 axis)
{
	p2Vec3 v = axis.Normalized();
	p2Mat33 rota(p2Vec3((cos(angle) + (v.x*v.x)*(1.0f - cos(angle))),
						   (v.y * v.z) * (1.0f - cos(angle)) + v.z * sin(angle),
						   (v.z * v.x) * (1.0f - cos(angle)) - (v.y * sin(angle))
						  ), 
				    p2Vec3(v.x * v.y * (1.0f - cos(angle)) - (v.z * sin(angle)), 
						   cos(angle) + (v.y * v.y) * (1.0f - cos(angle)), 
						   v.z * v.y * (1.0f - cos(angle)) + v.x * sin(angle)
					      ), 
					p2Vec3(v.x * v.z * (1.0f - cos(angle)) + v.y * sin(angle), 
						   v.y * v.z * (1.0f - cos(angle)) - v.x * sin(angle), 
						   cos(angle)+ ((v.z * v.z) * (1.0f - cos(angle)))
					));

	return rota;
}

p2Mat33 p2Mat33::Invert()
{
	float det = 1 / ((v1.x * v2.y * v3.z) + (v1.z * v2.x * v3.y) + (v1.y * v2.z * v3.x) - (v1.z * v2.y * v3.x) - (v1.x * v2.z * v3.y) - (v1.y * v2.x * v3.z));

	p2Mat33 signe(p2Vec3(1, -1, 1), p2Vec3(-1, 1, -1), p2Vec3(1, -1, 1));
	p2Mat33 adjacent(p2Vec3((v2.y * v3.z - v2.z * v3.y) * signe.v1.x, 
							   (v1.y * v3.z - v1.z * v3.y) * signe.v1.y,
							   (v1.y * v2.z - v1.z * v2.y) * signe.v1.z),
						p2Vec3((v2.x * v3.z - v2.z * v3.x) * signe.v2.x,
							   (v1.x * v3.z - v1.z * v3.x) * signe.v2.y,
							   (v1.x * v2.z - v1.z * v2.x) * signe.v2.z),
						p2Vec3((v2.x * v3.y - v2.y * v3.x) * signe.v3.x,
							   (v1.x * v3.y - v1.y * v3.x) * signe.v3.y,
							   (v1.x * v2.y - v1.y * v2.x) * signe.v3.z));
	
	return p2Mat33(adjacent * det);
}

void p2Mat33::Show()
{
	std::cout << "| " << v1.x << " " << v2.x << " " << v3.x << " | \n| " << v1.y << " " << v2.y << " " << v3.y << " | \n| " << v1.z << " " << v2.z << " " << v3.z << " |" << "\n \n";
}
