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

#include <p2vector.h>
#include <math.h>
#include <iostream>

p2Vec2::p2Vec2()
{
}

p2Vec2::p2Vec2(float x, float y):x(x), y(y)
{
}

void p2Vec2::Set(float x, float y)
{
	this->x = x;
	this->y = y;
}

p2Vec2 p2Vec2::operator+(p2Vec2 v)
{
	return p2Vec2(x + v.x, y + v.y);
}

p2Vec2 p2Vec2::operator+=(p2Vec2 v)
{
	return *this = *this + v;
}

p2Vec2 p2Vec2::operator-(p2Vec2 v)
{
	return p2Vec2(x - v.x, y - v.y);
}

p2Vec2 p2Vec2::operator-=(p2Vec2 v)
{
	return *this = *this - v;
}

p2Vec2 p2Vec2::operator/(float f)
{
	return p2Vec2(x / f, y / f);
}

p2Vec2 p2Vec2::operator*(float f)
{
	return p2Vec2(x * f, y * f);
}

p2Vec2 p2Vec2::operator*=(float f)
{
	return *this = *this * f;
}

float p2Vec2::Dot(p2Vec2 v1, p2Vec2 v2)
{
	return (v1.x * v2.x) + (v1.y * v2.y);
}

p2Vec3 p2Vec2::Cross(p2Vec2 v1, p2Vec2 v2)
{
	return p2Vec3(0,
				  0,
				  (v1.x * v2.y) - (v1.y * v2.x));
}

float p2Vec2::GetMagnitude()
{
	return sqrt((x*x) + (y*y));
}

p2Vec2 p2Vec2::Normalized()
{
	return p2Vec2(x/GetMagnitude(), y/GetMagnitude());
}

void p2Vec2::Normalize()
{
	float magnitude = GetMagnitude();
	x /= magnitude;
	y /= magnitude;
}

p2Vec3 p2Vec2::to3()
{
	return p2Vec3(x, y, 0);
}

void p2Vec2::Show()
{
	std::cout << "(" << x << ", " << y << ")" << "\n";
}

p2Vec3::p2Vec3()
{
}

p2Vec3::p2Vec3(float x, float y, float z) : x(x), y(y), z(z)
{
}

p2Vec3 p2Vec3::operator+(p2Vec3 v)
{
	return p2Vec3(x + v.x, y + v.y, z + v.z);
}

p2Vec3 p2Vec3::operator+=(p2Vec3 v)
{
	return *this = *this + v;
}

p2Vec3 p2Vec3::operator-(p2Vec3 v)
{
	return p2Vec3(x - v.x, y - v.y, z - v.z);
}

p2Vec3 p2Vec3::operator-=(p2Vec3 v)
{
	return *this = *this - v;
}

p2Vec3 p2Vec3::operator /(float f)
{
	return p2Vec3(x / f, y / f, z / f);
}

p2Vec3 p2Vec3::operator *(float f)
{
	return p2Vec3(x * f, y * f, z * f);
}

float p2Vec3::Dot(p2Vec3 v1, p2Vec3 v2)
{
	return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
}

p2Vec3 p2Vec3::Cross(p2Vec3 v1, p2Vec3 v2)
{
	return p2Vec3((v1.y * v2.z) - (v1.z * v2.y),
				  (v1.z * v2.x) - (v1.x * v2.z),
				  (v1.x * v2.y) - (v1.y * v2.x));
}

p2Vec3 p2Vec3::Lerp(p2Vec3 v1, p2Vec3 v2, float ratio)
{
	p2Vec3 vec12 = v2 - v1;
	return p2Vec3(v1.x + vec12.x * ratio, 
		          v1.y + vec12.y * ratio, 
		          v1.z + vec12.z * ratio);
}

p2Vec3 p2Vec3::Proj(p2Vec3 v1, p2Vec3 v2)
{
	float f = Dot(v1, v2) / (v2.GetMagnitude() * v2.GetMagnitude());
	return p2Vec3(v2 * f);
}

p2Vec3 p2Vec3::Refl(p2Vec3 inDir, p2Vec3 normal)
{
	return inDir - normal.Normalized() * 2 * Dot(inDir, normal.Normalized());
}

float p2Vec3::AnglesBetween(p2Vec3 v1, p2Vec3 v2)
{
	return acos(Dot(v1, v2) / (v1.GetMagnitude() * v2.GetMagnitude()))* M_PI/ 180;
}

float p2Vec3::GetMagnitude()
{
	return sqrt((x*x) + (y*y) + (z*z));
}

p2Vec3 p2Vec3::Normalized()
{
	return p2Vec3(x / GetMagnitude(), y / GetMagnitude(), z / GetMagnitude());
}

void p2Vec3::Normalize()
{
	float magnitude = GetMagnitude();
	x /= magnitude;
	y /= magnitude;
	z /= magnitude;
}

void p2Vec3::Show()
{
	std::cout << "(" << x << ", " << y << ", " << z << ")" << "\n";
}
