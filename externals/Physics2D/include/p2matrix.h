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

struct p2Matrix33;

/**
* \brief Matrix22 class
*/
struct p2Matrix22
{
	//Constructors
	p2Matrix22();

	p2Matrix22(p2Vec2 v1, p2Vec2 v2);

	//Operators
	p2Matrix22 operator+(p2Matrix22 m);
	p2Matrix22 operator+=(p2Matrix22 m);
	p2Matrix22 operator-(p2Matrix22 m);
	p2Matrix22 operator-=(p2Matrix22 m);
	p2Matrix22 operator*(p2Vec2 v);
	p2Matrix22 operator*=(p2Vec2 v);
	p2Matrix22 operator*(p2Matrix22 m);
	p2Matrix22 operator*=(p2Matrix22 m);

	//function
	/**
	* \brief generate tthe rotation matrix with the given angle (in radian)
	*/
	static p2Matrix22 RotationMatrix(float angle);

	/**
	* \brief calcul the matrix's inverse
	*/
	p2Matrix22 Inverse();

	/**
	* \brief display the matrix in the console
	*/
	void Show();

	//Variables
	p2Vec2 v1 = p2Vec2(0.0f, 0.0f);;
	p2Vec2 v2 = p2Vec2(0.0f, 0.0f);;
};

/**
* \brief Matrix33 class
*/
struct p2Matrix33
{
	
};

#endif