#pragma once
#include "Vec4.h"
#include "Vec3.h"

#include <stdarg.h>

struct Box {

	float left;
	float right;
	float bottom;
	float top;
	float back;
	float front;	

};

struct BoundingBox {

	Vec3 backTopLeft;
	Vec3 backTopRight;
	Vec3 backBottomLeft;
	Vec3 backBottomRight;
	Vec3 frontTopLeft;
	Vec3 frontTopRight;
	Vec3 frontBottomLeft;
	Vec3 frontBottomRight;

};

struct Plane {

	Vec3 topLeft;
	Vec3 topRight;
	Vec3 bottomRight;
	Vec3 bottomLeft;

};

struct Ray {

	Vec3 origin;
	Vec3 direction;

};

struct Triangle {

	Vec3 v1;
	Vec3 v2;
	Vec3 v3;

};

template <class Vertex>
BoundingBox GenerateBoundingBox(int nVertices, Vertex* pVertex, int positionOffset)
{

	// initialize the bounding box with values that will be overwritten
	Box b;
	b.left = 1000000;
	b.right = -100000;

	b.bottom = 1000000;
	b.top = -1000000;

	b.back = 1000000;
	b.front = -1000000;

	// assemble the bounding box using the min and max
	// of each coordinate dimension
	Vertex* last = pVertex + nVertices;
	for ( Vertex* v = pVertex; v != last; v++ ) {

		Vec3& pos = *(Vec3*)((float*)v + positionOffset);

		if ( pos.x < b.left )
			b.left = pos.x;
		if ( pos.x > b.right )
			b.right = pos.x;

		if ( pos.y < b.bottom )
			b.bottom = pos.y;
		if ( pos.y > b.top )
			b.top = pos.y;

		if ( pos.z < b.back )
			b.back = pos.z;
		if ( pos.z > b.front )
			b.front = pos.z;

	}

	// assemble the vertices of the bounding box
	BoundingBox bb;
	bb.backTopLeft = { b.left, b.top, b.back };
	bb.backTopRight = { b.right, b.top, b.back };
	bb.backBottomLeft = { b.left, b.bottom, b.back };
	bb.backBottomRight = { b.right, b.bottom, b.back };

	bb.frontTopLeft = { b.left, b.top, b.front };
	bb.frontTopRight = { b.right, b.top, b.front };
	bb.frontBottomLeft = { b.left, b.bottom, b.front };
	bb.frontBottomRight = { b.right, b.bottom, b.front };

	return bb;

}

bool TestIntersectTriangle(const Ray& ray, const Triangle& triangle);
bool TestIntersectSphere(const Ray& ray, const Vec3& center, float radius);
bool TestIntersectBox(const Ray& ray, const BoundingBox& boundingBox);
bool TestIntersectAxisAlignedBox(const Ray& ray, const Box& box);
bool TestIntersectPlane(const Ray& ray, const Plane& plane);

inline bool TestPointInsideBox(const Vec3& point, const Box& box)
{
	return point.x >= box.left && point.x < box.right &&
		point.y >= box.bottom && point.y < box.top &&
		point.z >= box.back && point.z < box.front;
}

inline bool TestTriangleInsideBox(const Triangle& t, const Box& box)
{
	return TestPointInsideBox(t.v1, box) && TestPointInsideBox(t.v2, box) && TestPointInsideBox(t.v3, box);
}

struct Frustum {

	float near;
	float far;

	float left;
	float right;

	float bottom;
	float top;

};

struct Sphere {

	float radius;

	int nTriangles;
	int nVertices;

	int* pIndices = nullptr;
	Vec4* pVertices = nullptr;

	Sphere(int resolution, float radius);
	~Sphere();

};


