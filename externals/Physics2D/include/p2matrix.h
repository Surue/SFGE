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

#ifndef SFGE_P2MATRIX_H
#define SFGE_P2MATRIX_H

#include <p2vector.h>

struct p2Mat33;

/**
* \brief Matrix22 class
*/
struct p2Mat22
{
	//Constructors
	p2Mat22();
	p2Mat22(p2Vec2 v1, p2Vec2 v2);

	//Operators
	p2Mat22 operator+(const p2Mat22 m) const;
	p2Mat22 operator+=(const p2Mat22 m);
	p2Mat22 operator-(const p2Mat22 m) const;
	p2Mat22 operator-=(const p2Mat22 m);
	p2Vec2 operator*(const p2Vec2 v) const;
	p2Mat22 operator*(const p2Mat22 m) const;
	p2Mat22 operator*=(const p2Mat22 m);

	//Methods
	/**
	* \brief generate tthe rotation matrix with the given angle (in radian). Counter clock wise
	*/
	static const p2Mat22 RotationMatrix(float angle);
	/**
	* \brief Calculate the invert of the 2x2 matrix
	*/
	p2Mat22 Invert();

	/**
	* \brief display the matrix in the console
	*/
	void Show() const;
	/**
	* \brief return the identary matrix22
	*/
	static p2Mat22 Identity();

	//Variables
	p2Vec2 v1 = p2Vec2(0.0f, 0.0f);;
	p2Vec2 v2 = p2Vec2(0.0f, 0.0f);;
};

/**
* \brief Matrix33 class
*/
struct p2Mat33
{
	//Constructors
	p2Mat33();
	p2Mat33(p2Vec3 v1, p2Vec3 v2, p2Vec3 v3);

	//Variables
	p2Mat33 operator+(const p2Mat33 m) const;
	p2Mat33 operator+=(const p2Mat33 m);
	p2Mat33 operator-(const p2Mat33 m) const;
	p2Mat33 operator-=(const p2Mat33 m);
	p2Vec3 operator*(const p2Vec3 v) const;
	p2Mat33 operator*(const p2Mat33 m)const;
	p2Mat33 operator*=(const p2Mat33 m);
	p2Mat33 operator*(float f) const;
	p2Mat33 operator*=(float f);

	//Methods
	/**
	* \brief generate the rotation matrix with the given angle (in radian) and the axis
	*/
	static p2Mat33 RotationMatrix(float angle, const p2Vec3 axis);
	/**
	* \brief Calculate the invert of the 3x3 matrix
	*/
	p2Mat33 Invert();
	/**
	* \brief return the transposed matrix
	*/
	p2Mat33 Transposed();
	/**
	* \brief display the matrix in the console
	*/
	void Show() const;
	/**
	* \brief return the identary matrix33
	*/
	static p2Mat33 Identity();

	//Variables
	p2Vec3 v1 = p2Vec3(0.0f, 0.0f, 0.0f);
	p2Vec3 v2 = p2Vec3(0.0f, 0.0f, 0.0f);
	p2Vec3 v3 = p2Vec3(0.0f, 0.0f, 0.0f);
};

#endif