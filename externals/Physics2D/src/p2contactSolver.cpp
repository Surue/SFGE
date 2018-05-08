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

#include <p2contactSolver.h>
#include <p2contact.h>
#include <p2Matrix.h>

#include <iostream>
#include <algorithm>

bool SAT::CheckCollisionSAT(p2Contact * contact, p2Manifold& manifold)
{
	switch (contact->GetColliderA()->GetShapeType()) {
	case p2ColliderDef::ShapeType::CIRCLE:
		switch (contact->GetColliderB()->GetShapeType()) {
		case p2ColliderDef::ShapeType::CIRCLE:
			return CheckCollisionCircles(contact, manifold);
			break;

		case p2ColliderDef::ShapeType::POLYGON:
			return CheckCollisionPolygonCircle(contact, manifold);
			break;
		}
		break;

	case p2ColliderDef::ShapeType::POLYGON:
		switch (contact->GetColliderB()->GetShapeType()) {
		case p2ColliderDef::ShapeType::CIRCLE:
			return CheckCollisionPolygonCircle(contact, manifold);
			break;

		case p2ColliderDef::ShapeType::POLYGON:
			return CheckCollisionPolygons(contact, manifold);
			break;
		}
		break;
	}
	return false;
}

bool SAT::CheckCollisionCircles(p2Contact * contact, p2Manifold& manifold)
{
	p2Vec2 distance = (contact->GetColliderB()->GetPosition() - contact->GetColliderA()->GetPosition());

	float radiusA = static_cast<p2CircleShape*>(contact->GetColliderA()->GetShape())->GetRadius();
	float radiusB = static_cast<p2CircleShape*>(contact->GetColliderB()->GetShape())->GetRadius();

	p2Vec2 positionA = contact->GetColliderA()->GetPosition();
	p2Vec2 positionB = contact->GetColliderB()->GetPosition();

	float radiusTotal = radiusA + radiusB;
	
	if (distance.GetMagnitude() < radiusTotal) {
		manifold.normal = distance.Normalized();
		manifold.penetration = p2Vec2::Dot((positionA + manifold.normal * radiusA) - (positionB - manifold.normal * radiusB), manifold.normal);
		manifold.ShouldResolve = contact->ShouldResolveCollision();
		manifold.contactPoint = ((positionA + manifold.normal * radiusA) + (positionB - manifold.normal * radiusB)) * 0.5f;
		return true;
	}
	else {
		return false;
	}
}

bool SAT::CheckCollisionPolygons(p2Contact * contact, p2Manifold& manifold)
{
	p2Collider* colliderA = contact->GetColliderA();
	p2Collider* colliderB = contact->GetColliderB();

	p2PolygonShape* polygonA;
	p2Vec2 polygonAPosition;
	float polygonAAngle;

	//Get all vectors and normal from rectB

	p2PolygonShape* polygonB;
	p2Vec2 polygonBPosition;
	float polygonBAngle;

	polygonA = static_cast<p2PolygonShape*>(colliderA->GetShape());
	polygonAPosition = colliderA->GetPosition();
	polygonAAngle = colliderA->GetBody()->GetAngle();

	polygonB = static_cast<p2PolygonShape*>(colliderB->GetShape());
	polygonBPosition = colliderB->GetPosition();
	polygonBAngle = colliderB->GetBody()->GetAngle();

	//Get all vectors and normal from rect
	std::vector<p2Vec2> polygonAVectors;
	std::vector<p2Vec2> polygonANormals;

	polygonAVectors.resize(polygonA->GetVerticesCount());
	polygonANormals.resize(polygonA->GetVerticesCount());
	polygonAVectors = polygonA->GetVerticesWorld(polygonAPosition, polygonAAngle);

	std::vector<p2Vec2> polygonAVectorsVertices;
	polygonAVectorsVertices.resize(polygonA->GetVerticesCount());
	polygonA->GetVectorsVertices(polygonAVectorsVertices, polygonAPosition, polygonAAngle);

	polygonA->GetNormals(polygonANormals, polygonAVectorsVertices);

	//Get all vectors and normal from polygon
	std::vector<p2Vec2> polygonBVectors;
	std::vector<p2Vec2> polygonBNormals;

	polygonBVectors.resize(polygonB->GetVerticesCount());
	polygonBNormals.resize(polygonB->GetVerticesCount());
	polygonBVectors = polygonB->GetVerticesWorld(polygonBPosition, polygonBAngle);

	std::vector<p2Vec2> polygonBVectorsVertices;
	polygonBVectorsVertices.resize(polygonB->GetVerticesCount());
	polygonB->GetVectorsVertices(polygonBVectorsVertices, polygonBPosition, polygonBAngle);

	polygonB->GetNormals(polygonBNormals, polygonBVectorsVertices);

	//Get MinMax projection on each normal
	bool isSeparated = false;

	p2Vec2 normalMinimal;
	float mtv = std::numeric_limits<float>::max();

	for (int i = 0; i < polygonA->GetVerticesCount(); i++) {
		p2Vec2 minMaxA = GetMinMaxProj(polygonAVectors, polygonANormals[i]);
		p2Vec2 minMaxB = GetMinMaxProj(polygonBVectors, polygonANormals[i]);

		float minA = minMaxA.x; float maxA = minMaxA.y;
		float minB = minMaxB.x; float maxB = minMaxB.y;

		isSeparated = maxA < minB || maxB < minA;
		if (isSeparated) {
			break;
		}

		if (mtv > maxA - minB) {
			mtv = maxA - minB;
			normalMinimal = polygonANormals[i];
		}
		else if (mtv > maxB - minA) {
			mtv = maxB - minA;
			normalMinimal = polygonANormals[i] * -1;
		}
	}
	if (!isSeparated) {
		for (int i = 0; i < polygonB->GetVerticesCount(); i++) {
			p2Vec2 minMaxA = GetMinMaxProj(polygonAVectors, polygonBNormals[i]);
			p2Vec2 minMaxB = GetMinMaxProj(polygonBVectors, polygonBNormals[i]);

			float minA = minMaxA.x; float maxA = minMaxA.y;
			float minB = minMaxB.x; float maxB = minMaxB.y;

			isSeparated = maxA < minB || maxB < minA;
			if (isSeparated) {
				break;
			}

			if (mtv > maxA - minB) {
				mtv = maxA - minB;
				normalMinimal = polygonBNormals[i];
			}
			else if (mtv > maxB - minA) {
				mtv = maxB - minA;
				normalMinimal = polygonBNormals[i] * -1;
			}
		}
	}

	if (!isSeparated) {
		manifold.penetration = mtv;
		manifold.normal = normalMinimal;
		manifold.ShouldResolve = contact->ShouldResolveCollision();

		if (manifold.ShouldResolve) {
			manifold.contactPoint = FindContactPoint(contact, manifold);
		}
	}

	return !isSeparated;
}

bool SAT::CheckCollisionPolygonCircle(p2Contact * contact, p2Manifold& manifold)
{
	//Variables
	p2Collider* colliderA = contact->GetColliderA();
	p2Collider* colliderB = contact->GetColliderB();

	p2PolygonShape* polygon;
	p2Vec2 polygonPosition;
	float polygonAngle;
	p2CircleShape* circle;
	p2Vec2 circlePosition;

	bool flip = false;

	//Associate variables
	if (colliderA->GetShapeType() == p2ColliderDef::ShapeType::POLYGON) {
		polygon = static_cast<p2PolygonShape*>(colliderA->GetShape());
		polygonPosition = colliderA->GetPosition();
		polygonAngle = colliderA->GetBody()->GetAngle();

		circle = static_cast<p2CircleShape*>(colliderB->GetShape());
		circlePosition = colliderB->GetPosition();

		flip = true;
	}
	else {
		polygon = static_cast<p2PolygonShape*>(colliderB->GetShape());
		polygonPosition = colliderB->GetPosition();
		polygonAngle = colliderB->GetBody()->GetAngle();

		circle = static_cast<p2CircleShape*>(colliderA->GetShape());
		circlePosition = colliderA->GetPosition();
	}

	//Get closet point of polygon to the circle
	std::vector<p2Vec2> vectorsPolygon;
	vectorsPolygon.resize(polygon->GetVerticesCount());
	vectorsPolygon = polygon->GetVerticesWorld(polygonPosition, polygonAngle);
	
	int indexClosestVertice = 0;
	float minDistance = (circlePosition - vectorsPolygon[0]).GetMagnitude();
	for (int i = 1; i < vectorsPolygon.size(); i++) {
		float curDistance = (circlePosition - vectorsPolygon[i]).GetMagnitude();

		if (curDistance < minDistance) {
			minDistance = curDistance;
			indexClosestVertice = i;
		}
	}

	//Get normals from polygon
	std::vector<p2Vec2> polygonNormals;

	polygonNormals.resize(polygon->GetVerticesCount());

	std::vector<p2Vec2> polygonVectorsVertices;
	polygonVectorsVertices.resize(polygon->GetVerticesCount());
	polygon->GetVectorsVertices(polygonVectorsVertices, polygonPosition, polygonAngle);

	polygon->GetNormals(polygonNormals, polygonVectorsVertices);

	//Make axis from closet point to center of circle and add this to normals to check
	p2Vec2 polygon2circle = circlePosition - vectorsPolygon[indexClosestVertice];

	//Add normal from circle to polygon
	polygonNormals.push_back(polygon2circle.Normalized());

	p2Vec2 normalMinimal;
	float mtv = std::numeric_limits<float>::max();

	//Proj
	bool isSeparated = false;
	for (int i = 0; i < polygonNormals.size(); i++) {
		p2Vec2 minMaxA = GetMinMaxProj(vectorsPolygon, polygonNormals[i]);
		float minA = minMaxA.x; float maxA = minMaxA.y;

		float minB = p2Vec2::Dot(circlePosition, polygonNormals[i]) - circle->GetRadius(); 
		float maxB = p2Vec2::Dot(circlePosition, polygonNormals[i]) + circle->GetRadius();

		isSeparated = maxA < minB || maxB < minA;

		if (isSeparated) {
			break;
		}

		if (mtv > maxA - minB) {
			mtv = maxA - minB;
			normalMinimal = polygonNormals[i] * -1;
		}
		else if (mtv > maxB - minA) {
			mtv = maxB - minA;
			normalMinimal = polygonNormals[i];
		}
	}

	if (!isSeparated) {
		if (flip) {
			manifold.contactPoint = circlePosition + normalMinimal * circle->GetRadius();
			normalMinimal *= -1;
		}
		else {
			manifold.contactPoint = circlePosition + normalMinimal * circle->GetRadius();
		}

		manifold.penetration = mtv;
		manifold.normal = normalMinimal;
		manifold.ShouldResolve = contact->ShouldResolveCollision();
	}

	return !isSeparated;
}

bool SAT::CheckCollisionLineCircle(p2Contact * contact, p2Manifold & manifold)
{
	p2Collider* colliderA = contact->GetColliderA();
	p2Collider* colliderB = contact->GetColliderB();

	p2CircleShape* circle;
	p2LineShape* line;

	p2Vec2 circlePosition;
	p2Vec2 posA, posB;
	float radius;

	if (colliderA->GetShapeType() == p2ColliderDef::ShapeType::CIRCLE) {
		circle = static_cast<p2CircleShape*>(colliderA->GetShape());
		circlePosition = colliderA->GetPosition();

		line = static_cast<p2LineShape*>(colliderB->GetShape());
	}
	else {
		circle = static_cast<p2CircleShape*>(colliderB->GetShape());
		circlePosition = colliderB->GetPosition();

		line = static_cast<p2LineShape*>(colliderA->GetShape());
	}

	radius = circle->GetRadius();

	posA = line->m_PosA;
	posB = line->m_PosB;

	p2Vec2 normal = (posB - posA).Normal().Normalized();

	p2Vec2 lineToCircle = circlePosition - posA;

	if (p2Vec2::Proj(lineToCircle, normal).GetMagnitude() < radius) {
		p2Vec2 direction = (posB - posA).Normalized();

		float t = p2Vec2::Dot(direction, circlePosition - posA);

		p2Vec2 pointOnLine = direction * t + posA;

		float distance = sqrt((radius * radius) - (pointOnLine - circlePosition).GetMagnitude() * (pointOnLine - circlePosition).GetMagnitude());

		p2Vec2 closestPoint = direction * (t - distance) + posA;

			
		manifold.penetration = radius - (circlePosition - closestPoint).GetMagnitude();
		manifold.normal = (posB - posA).Normal().Normalized();
		manifold.ShouldResolve = false;
		manifold.contactPoint = closestPoint;
		
		return true;
	}
	else {
		return false;
	}
}

bool SAT::CheckCollisionLinePolygon(p2Contact * contact, p2Manifold & manifold)
{
	p2Collider* colliderA = contact->GetColliderA();
	p2Collider* colliderB = contact->GetColliderB();

	p2PolygonShape* polygon;
	p2LineShape* line;

	p2Vec2 posA, posB;
	p2Vec2 polygonPosition;
	float polygonAngle;

	if (colliderA->GetShapeType() == p2ColliderDef::ShapeType::POLYGON) {
		polygon = static_cast<p2PolygonShape*>(colliderA->GetShape());
		polygonPosition = colliderA->GetPosition();
		polygonAngle = colliderA->GetBody()->GetAngle();

		line = static_cast<p2LineShape*>(colliderB->GetShape());
	}
	else {
		polygon = static_cast<p2PolygonShape*>(colliderB->GetShape());
		polygonPosition = colliderB->GetPosition();
		polygonAngle = colliderB->GetBody()->GetAngle();

		line = static_cast<p2LineShape*>(colliderA->GetShape());
	}

	posA = line->m_PosA;
	posB = line->m_PosB;

	std::vector<p2Vec2> vertices;
	vertices.resize(polygon->GetVerticesCount());
	vertices = polygon->GetVerticesWorld(polygonPosition, polygonAngle);

	std::vector<p2Vec2> closestPoints;

	bool isTouching = false;

	for (int i = 0; i < vertices.size(); i++) {
		p2Vec2 pos2 = vertices[i];
		int next = i + 1;
		if (next == vertices.size()) next = 0;
		p2Vec2 pos1 = vertices[next];

		float divisor = (posB.y - posA.y) * (pos2.x - pos1.x) - (posB.x - posA.x) * (pos2.y - pos1.y);

		float u1 = ((posB.x - posA.x) * (pos1.y - posA.y) - (posB.y - posA.y) * (pos1.x - posA.x)) / divisor;
		float u2 = ((pos2.x - posA.x) * (pos1.y - posA.y) - (pos2.y - posA.y) * (pos1.x - posA.x)) / divisor;

		if (u1 >= 0 && u1 <= 1 && u2 >= 0 && u2 <= 1) {
			p2Vec2 intersection = pos1 + ((pos2 - pos1) * u1);
			closestPoints.push_back(intersection);
			isTouching = true;

			//if second point found, it's useless to continue
			if (closestPoints.size() == 2) {
				break;
			}
		}
	}
	
	if (isTouching) {
		if (closestPoints.size() == 2) {
			if ((posA - closestPoints[0]).GetMagnitude() < (posA - closestPoints[1]).GetMagnitude()) {
				manifold.contactPoint = closestPoints[0];
			}
			else {
				manifold.contactPoint = closestPoints[1];
			}
		}
		else {
			manifold.contactPoint = closestPoints[0];
		}

		manifold.normal = (polygonPosition - manifold.contactPoint).Normalized();
		manifold.ShouldResolve = false;
		return true;
	}

	return false;
}

p2Vec2 SAT::FindContactPoint(const p2Contact* contact, const p2Manifold & manifold)
{
	p2Collider* colliderA = contact->GetColliderA();
	p2Collider* colliderB = contact->GetColliderB();

	std::vector<p2Vec2> cornersA;
	std::vector<p2Vec2> cornersB;

	bool flip = false;

	//We now this function is only called if we have rect or polygon
	if (colliderA->GetShapeType() == p2ColliderDef::ShapeType::POLYGON) {
		p2PolygonShape* shapeA = static_cast<p2PolygonShape*>(colliderA->GetShape());
		cornersA.resize(shapeA->GetVerticesCount());
		cornersA = shapeA->GetVerticesWorld(colliderA->GetPosition(), colliderA->GetBody()->GetAngle());
	}
	else {
		std::cout << "ERROR: FIND CONTACT POINT COLLIDER A\n";
		return manifold.contactPoint;
	}

	if (colliderB->GetShapeType() == p2ColliderDef::ShapeType::POLYGON) {
		p2PolygonShape* shapeB = static_cast<p2PolygonShape*>(colliderB->GetShape());
		cornersB.resize(shapeB->GetVerticesCount());
		cornersB = shapeB->GetVerticesWorld(colliderB->GetPosition(), colliderB->GetBody()->GetAngle());
	}
	else {
		std::cout << "ERROR: FIND CONTACT POINT COLLIDER B\n";
		return manifold.contactPoint;
	}

	p2Vec2 normalContact = manifold.normal;

	p2Edge closestA = FindClosestEdge(cornersA, normalContact);
	p2Edge closestB = FindClosestEdge(cornersB, normalContact * (-1));

	//Find the reference and incident edge
	p2Edge ref;
	p2Edge inc;

	if (abs(p2Vec2::Dot(closestA.vector, normalContact)) <= abs(p2Vec2::Dot(closestB.vector, normalContact))) {
		ref = closestA;
		inc = closestB;
	}
	else {
		ref = closestB;
		inc = closestA;
		flip = true;
	}
	
	//Start clipping the points
	p2Vec2 refVector = ref.vector.Normalized();

	//Create the max points
	float offset1 = p2Vec2::Dot(refVector, ref.pointA);;
	float offset2 = p2Vec2::Dot(refVector, ref.pointB);

	//Clip other point with the limit
	std::vector<p2Vec2> clippedPoints;
	clippedPoints.resize(2);

	clippedPoints = ClipPoints(inc.pointA, inc.pointB, refVector, offset1);

	if (clippedPoints.size() < 2) {
		std::cout << "MOTHERFUCKER 1\n";
		return p2Vec2();
	}

	//Same clipping to the other direction
	clippedPoints = ClipPoints(clippedPoints[0], clippedPoints[1], refVector * (-1), -offset2);
	if (clippedPoints.size() < 2) {
		std::cout << "MOTHERFUCKER 2\n";
		return p2Vec2();
	}

	p2Vec2 refNormal = ref.vector.Normal().Normalized();

	if (flip) refNormal *= -1;

	float max = p2Vec2::Dot(refNormal, ref.max);

	if (p2Vec2::Dot(refNormal, clippedPoints[0]) - max >= 0.0f) {
		if (p2Vec2::Dot(refNormal, clippedPoints[1]) - max >= 0.0f) {
			return (clippedPoints[0] + clippedPoints[1]) / 2;
		}
		else {
			return clippedPoints[0];
		}
	}
	return clippedPoints[1];
}

p2Edge SAT::FindClosestEdge(std::vector<p2Vec2> const vertices, p2Vec2 const normal)
{
	int index = 0;
	float maxProj = p2Vec2::Dot(vertices[index], normal);

	//Find the farthest vertex from the normal
	for (int i = 1; i < vertices.size(); i++) {
		float curProj = p2Vec2::Dot(vertices[i], normal);

		if (curProj > maxProj) {
			maxProj = curProj;
			index = i;
		}
	}
	
	//Check the left and right edge from the previous one to see wich one if the most perpendicular to the normal
	p2Vec2 v = vertices[index];
	p2Vec2 v1;
	if (index == vertices.size() - 1) {
		v1 = vertices[0];
	}
	else {
		v1 = vertices[index + 1];
	}
	p2Vec2 v2;
	if (index == 0) {
		v2 = vertices[vertices.size() - 1];
	}
	else {
		v2 = vertices[index - 1];
	}

	p2Vec2 left = (v - v1).Normalized();
	p2Vec2 right = (v - v2).Normalized();

	p2Edge closestEdge;

	if (p2Vec2::Dot(right, normal) <= p2Vec2::Dot(left, normal)) {
		closestEdge.max = v2;
		closestEdge.pointA = v2;
		closestEdge.pointB = v;
		closestEdge.vector = v - v2;
	}
	else {
		closestEdge.max = v;
		closestEdge.pointA = v;
		closestEdge.pointB = v1;
		closestEdge.vector = v1 - v;
	}

	return closestEdge;
}

std::vector<p2Vec2> SAT::ClipPoints(p2Vec2 pointsA, p2Vec2 pointsB, p2Vec2 normal, float proj)
{
	std::vector<p2Vec2> clippedPoints;
	//Projection of points allong the normal
	float d1 = p2Vec2::Dot(pointsA, normal) - proj;
	float d2 = p2Vec2::Dot(pointsB, normal) - proj;
	
	//If the points are past the proj along n
	if (d1 >= 0.0f) clippedPoints.push_back(pointsA);
	if (d2 >= 0.0f) clippedPoints.push_back(pointsB);

	//If they are on opposing side, compute correct point to clip the point to the correct position
	if (d1 * d2 < 0.0) {
		p2Vec2 correctedPoint = pointsB - pointsA;

		float u = d1 / (d1 - d2);

		correctedPoint *= u;

		correctedPoint += pointsA;

		clippedPoints.push_back(correctedPoint);
	}

	return clippedPoints;
}

p2Vec2 SAT::GetMinMaxProj(p2Vec2 proj[], int sizeArray, p2Vec2 axis)
{
	float minProj = p2Vec2::Dot(proj[0], axis);
	float maxProj = p2Vec2::Dot(proj[0], axis);

	for (int i = 1; i < sizeArray; i++) {
		float curProj = p2Vec2::Dot(proj[i], axis);

		if (minProj > curProj) {
			minProj = curProj;
		}
		if (maxProj < curProj) {
			maxProj = curProj;
		}
	}

	return p2Vec2(minProj, maxProj);
}

p2Vec2 SAT::GetMinMaxProj(std::vector<p2Vec2>& proj, p2Vec2 axis)
{
	float minProj = p2Vec2::Dot(proj[0], axis);
	float maxProj = p2Vec2::Dot(proj[0], axis);

	for (int i = 1; i < proj.size(); i++) {
		float curProj = p2Vec2::Dot(proj[i], axis);

		if (minProj > curProj) {
			minProj = curProj;
		}
		if (maxProj < curProj) {
			maxProj = curProj;
		}
	}

	return p2Vec2(minProj, maxProj);
}
