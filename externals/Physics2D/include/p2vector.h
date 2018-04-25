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

#ifndef SFGE_P2VECTOR_H
#define SFGE_P2VECTOR_H

struct p2Vec3;

/**
* \brief Vector 2 class
*/
struct p2Vec2
{
	//Constructors
	p2Vec2();
	p2Vec2(float x, float y);

	void Set(float x, float y);

	//Operators
	p2Vec2 operator+(const p2Vec2 v) const;
	p2Vec2 operator+=(const p2Vec2 v);
	p2Vec2 operator-(const p2Vec2 v) const;
	p2Vec2 operator-=(const p2Vec2 v);
	p2Vec2 operator /(float f) const;
	p2Vec2 operator *(float f) const;
	p2Vec2 operator*=(float f);
	bool operator==(const p2Vec2 v);

	//Functions
	/**
	* \brief Dot product of two vectors
	*/
	static float Dot(const p2Vec2 const v1, const p2Vec2 const v2);
	/**
	* \brief Cross product of two vectors
	*/
	static p2Vec3 Cross(const p2Vec2 const v1, const p2Vec2 const v2);
	/**
	* \brief Compute angle between 2 vector
	*/
	static float p2Vec2::AnglesBetween(const p2Vec2 v1, const p2Vec2 v2);
	/**
	* \brief Project v1 on v2
	*/
	static p2Vec2 Proj(const p2Vec2 v1, const p2Vec2 v2);
	/**
	* \brief Calculate the magnitude of the p2Vec2
	*/
	float GetMagnitude() const;
	/**
	* \brief Calculate a normalized version of the p2Vec2
	*/
	p2Vec2 Normalized() const;
	/**
	* \brief Normalize the p2Vec2
	*/
	void Normalize();
	/**
	* \brief return a normal to the vector
	*/
	p2Vec2 Normal();
	/**
	* \brief 
	*/
	p2Vec3 to3() const;
	/**
	* \brief display vector on consol
	*/
	void Show() const;

	//Variables
	float x = 0.0f;
	float y = 0.0f;
};

/**
* \brief Vector 3 class
*/
struct p2Vec3
{
	//Constructors
	p2Vec3();
	p2Vec3(float x, float y, float z);

	//Operator
	p2Vec3 operator+(const p2Vec3 v) const;
	p2Vec3 operator+=(const p2Vec3 v);
	p2Vec3 operator-(const p2Vec3 v) const;
	p2Vec3 operator-=(const p2Vec3 v);
	p2Vec3 operator /(float f) const;
	p2Vec3 operator *(float f) const;

	//Function
	/**
	* \brief Dot product of two vectors
	*/
	static float Dot(const p2Vec3 v1, const p2Vec3 v2);
	/**
	* \brief Cross product of two vectors
	*/
	static p2Vec3 Cross(const p2Vec3 v1, const p2Vec3 v2);
	/**
	* \brief Lerp between two points
	*/
	static p2Vec3 Lerp(const p2Vec3 v1, const p2Vec3 v2, float ratio);
	/**
	* \brief Project v1 on v2
	*/
	static p2Vec3 Proj(const p2Vec3 v1, const p2Vec3 v2);
	/**
	* \brief Reflection of inDir by the normal
	*/
	static p2Vec3 Refl(const p2Vec3 inDir, const p2Vec3 normal);
	/**
	* \brief Compute angle between two vector
	*/
	static float AnglesBetween(const p2Vec3 v1, const p2Vec3 v2);
	/**
	* \brief Calculate the magnitude of the p2Vec3
	*/
	float GetMagnitude() const;
	/**
	* \brief Calculate a normalized version of the p2Vec2
	*/
	p2Vec3 Normalized() const;
	/**
	* \brief Normalize the p2Vec2
	*/
	void Normalize();
	/**
	* \brief display vector on consol
	*/
	void Show() const;

	//Variables
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
};

#endif