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

#define _CRTDBG_MAP_ALLOC
#include<iostream>
#include <crtdbg.h>
#ifdef _DEBUG
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

#define _USE_MATH_DEFINES

#include <p2matrix.h>
#include <iostream>
#include <cmath>

p2Mat22::p2Mat22()
{
}

p2Mat22::p2Mat22(p2Vec2 v1, p2Vec2 v2): v1(v1), v2(v2)
{
}

p2Mat22 p2Mat22::operator+(const p2Mat22 m) const
{
	return p2Mat22(v1 + m.v1, v2 + m.v2);
}

p2Mat22 p2Mat22::operator+=(const p2Mat22 m)
{
	return *this = *this + m;
}

p2Mat22 p2Mat22::operator-(const p2Mat22 m) const
{
	return p2Mat22(v1 - m.v1, v2 - m.v2);
}

p2Mat22 p2Mat22::operator-=(const p2Mat22 m)
{
	return *this = *this - m;
}

p2Vec2 p2Mat22::operator*(const p2Vec2 v) const
{
	return p2Vec2(v1.x * v.x + v2.x * v.y,
				  v1.y * v.x + v2.y * v.y);
}

p2Mat22 p2Mat22::operator*(const p2Mat22 m) const
{
	return p2Mat22(p2Vec2((v1.x * m.v1.x) + (v2.x * m.v1.y),
		  					 (v1.y * m.v1.x) + (v2.y * m.v1.y)), 
					  p2Vec2((v1.x * m.v2.x) + (v2.x * m.v2.y),
						     (v1.y * m.v2.x) + (v2.y * m.v2.y))
		              );
}

p2Mat22 p2Mat22::operator*=(p2Mat22 m)
{
	return *this = *this * m;
}

const p2Mat22 p2Mat22::RotationMatrix(float angle)
{
	return p2Mat22(p2Vec2(cos(angle), -sin(angle)),
					  p2Vec2(sin(angle), cos(angle)));
}

p2Mat22 p2Mat22::Invert()
{
	float det = 1 / ((v1.x * v2.y) - (v1.y * v2.x));

	if (det == 0) {
		return p2Mat22::Identity();
	}

	return p2Mat22(p2Vec2(v2.y * det, -v1.y * det), 
				   p2Vec2(-v2.x * det, v1.x * det));
}

void p2Mat22::Show() const
{
	std::cout << "| " << v1.x << " " << v2.x << " | \n| " 
					  << v1.y << " " << v2.y << " |" << "\n \n";
}

p2Mat22 p2Mat22::Identity()
{
	return p2Mat22(p2Vec2(1, 0), p2Vec2(0, 1));
}

p2Mat33::p2Mat33()
{
}

p2Mat33::p2Mat33(p2Vec3 v1, p2Vec3 v2, p2Vec3 v3): v1(v1), v2(v2), v3(v3)
{
}

p2Mat33 p2Mat33::operator+(const p2Mat33 m) const
{
	return p2Mat33(v1 + m.v1, v2 + m.v2, v3 + m.v3);
}

p2Mat33 p2Mat33::operator+=(const p2Mat33 m)
{
	return *this = *this + m;
}

p2Mat33 p2Mat33::operator-(const p2Mat33 m) const
{
	return p2Mat33(v1 - m.v1, v2 - m.v2, v3 - m.v3);
}

p2Mat33 p2Mat33::operator-=(const p2Mat33 m)
{
	return *this = *this - m;
}

p2Vec3 p2Mat33::operator*(const p2Vec3 v) const
{
	return p2Vec3((v1.x * v.x) + (v2.x * v.y) + (v3.x * v.z),
				  (v1.y * v.x) + (v2.y * v.y) + (v3.y * v.z),
			      (v1.z * v.x) + (v2.z * v.y) + (v3.z * v.z));
}

p2Mat33 p2Mat33::operator*(const p2Mat33 m) const
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

p2Mat33 p2Mat33::operator*=(const p2Mat33 m)
{
	return *this = *this * m;
}

p2Mat33 p2Mat33::operator*(float f) const
{
	return p2Mat33(v1 * f, v2 * f, v3 * f);
}

p2Mat33 p2Mat33::operator*=(float f)
{
	return *this = *this * f;
}

p2Mat33 p2Mat33::RotationMatrix(float angle, const p2Vec3 axis)
{
	p2Vec3 v = axis.Normalized();
	p2Mat33 rota(p2Vec3((cos(angle) + (v.x*v.x)*(1.0f - cos(angle))),
						(v.y * v.z) * (1.0f - cos(angle)) + v.z * sin(angle),
						(v.z * v.x) * (1.0f - cos(angle)) - (v.y * sin(angle))
					    ), 
				 p2Vec3((v.x * v.y * (1.0f - cos(angle)) - (v.z * sin(angle))), 
						(cos(angle) + (v.y * v.y) * (1.0f - cos(angle))), 
						(v.z * v.y * (1.0f - cos(angle)) + v.x * sin(angle))
					   ), 
			     p2Vec3((v.x * v.z * (1.0f - cos(angle)) + v.y * sin(angle)), 
						(v.y * v.z * (1.0f - cos(angle)) - v.x * sin(angle)), 
						(cos(angle)+ ((v.z * v.z) * (1.0f - cos(angle))))
					   )
				);

	return rota;
}

p2Mat33 p2Mat33::Invert()
{
	float det = 1 / ((v1.x * v2.y * v3.z) + (v1.z * v2.x * v3.y) + (v1.y * v2.z * v3.x) - (v1.z * v2.y * v3.x) - (v1.x * v2.z * v3.y) - (v1.y * v2.x * v3.z));

	if (det == 0) {
		return p2Mat33::Identity();
	}

	p2Mat33 signe(p2Vec3(1, -1, 1), p2Vec3(-1, 1, -1), p2Vec3(1, -1, 1));

	p2Mat33 matT = this->Transposed();

	p2Mat33 adjacent(p2Vec3((matT.v2.y * matT.v3.z - matT.v2.z * matT.v3.y) * signe.v1.x,
							(matT.v2.x * matT.v3.z - matT.v2.z * matT.v3.x) * signe.v1.y,
							(matT.v2.x * matT.v3.y - matT.v2.y * matT.v3.x) * signe.v1.z),
					 p2Vec3((matT.v1.y * matT.v3.z - matT.v1.z * matT.v3.y) * signe.v2.x,
							(matT.v1.x * matT.v3.z - matT.v1.z * matT.v3.x) * signe.v2.y,
							(matT.v1.x * matT.v3.y - matT.v1.y * matT.v3.x) * signe.v2.z),
					 p2Vec3((matT.v1.y * matT.v2.z - matT.v1.z * matT.v2.y) * signe.v3.x,
							(matT.v1.x * matT.v2.z - matT.v1.z * matT.v2.x) * signe.v3.y,
							(matT.v1.x * matT.v2.y - matT.v1.y * matT.v2.x) * signe.v3.z));
	
	return p2Mat33(adjacent * det);
}

p2Mat33 p2Mat33::Transposed()
{
	return p2Mat33(p2Vec3(v1.x, v2.x, v3.x), 
				   p2Vec3(v1.y, v2.y, v3.y), 
				   p2Vec3(v1.z, v2.z, v3.z));
}

void p2Mat33::Show() const
{
	std::cout << "| " << v1.x << " " << v2.x << " " << v3.x << " | \n| " 
					  << v1.y << " " << v2.y << " " << v3.y << " | \n| " 
					  << v1.z << " " << v2.z << " " << v3.z << " |" << "\n \n";
}

p2Mat33 p2Mat33::Identity()
{
	return p2Mat33(p2Vec3(1, 0, 0), p2Vec3(0, 1, 0), p2Vec3(0, 0, 1));
}
