#include "Shapes.h"
#include "Mat4.h"

#include <assert.h>

#define PI 3.14159265358979323846

static inline bool InsideOutsideTest(const Vec3* current, const Vec3* next, const Vec3& intersection, const Vec3& norm)
{
	Vec3 edge = *next - *current;
	Vec3 c = intersection - *current;
	return (norm * (edge % c) > 0);
}

static bool TestIntersectConvexPolygon(const Ray* ray, int nVertices, ...)
{

	// vertices must be passed as pointers to vec3

	assert(nVertices >= 3);

	va_list verts;
	va_start(verts, nVertices);

	const Vec3* v1 = va_arg(verts, const Vec3*);
	const Vec3* v2 = va_arg(verts, const Vec3*);
	const Vec3* v3 = va_arg(verts, const Vec3*);

	Vec3 norm = ((*v2 - *v1) % (*v3 - *v1)).Normalized();

	// the distance along the normal the plane of the shape
	// is from the origin
	float d = norm * *v1;

	// if the ray is perpendicular to the normal, it is not a hit
	if ( norm * ray->direction == 0 )
		return false;

	// the distance from the rays origin to its intersection
	// with the plane
	float t = (norm * ray->origin + d) / (norm * ray->direction);

	// if the intersection is behind the ray, ignore it
	if ( t < 0 )
		return false;

	// calculate the point of intersection with the plane
	Vec3 intersection = ray->origin + ray->direction * t;

	// perform the inside outside test
	const Vec3* current = v1;
	const Vec3* next = v2;

	if ( !InsideOutsideTest(current, next, intersection, norm) )
		return false;

	current = next;
	next = v3;

	if ( !InsideOutsideTest(current, next, intersection, norm) )
		return false;

	// loop through the rest of the vertices and perform the inside outside test
	for ( int i = 2; i < nVertices; ++i ) {

		if ( i == nVertices - 1 ) {
			// on the last vertex, connect back to the beginning

			current = next;
			next = v1;

			if ( !InsideOutsideTest(current, next, intersection, norm) )
				return false;

		}
		else {

			current = next;
			next = va_arg(verts, const Vec3*);

			if ( !InsideOutsideTest(current, next, intersection, norm) )
				return false;
		}
	}

	va_end(verts);
	return true;

}

bool TestIntersectTriangle(const Ray& ray, const Triangle& triangle)
{
	return TestIntersectConvexPolygon(&ray, 3, &triangle.v1, &triangle.v2, &triangle.v3);
}
bool TestIntersectSphere(const Ray& ray, const Vec3& center, float radius)
{
	// solve the quadratic equation for intersection of ray and sphere
	float a = ray.direction * ray.direction;
	float b = ray.direction * (ray.origin - center) * 2;
	float c = (ray.origin - center) * (ray.origin - center) - (radius * radius);

	float dis = b * b - 4 * a * c;
	if ( dis < 0 )
		return false;
	return true;
}
bool TestIntersectBox(const Ray& ray, const BoundingBox& boundingBox)
{
	// test all 6 planes of the box

	if ( TestIntersectConvexPolygon(
		&ray,
		4,
		&boundingBox.backTopLeft,
		&boundingBox.backTopRight,
		&boundingBox.backBottomRight,
		&boundingBox.backBottomLeft
	) ) return true;

	if ( TestIntersectConvexPolygon(
		&ray,
		4,
		&boundingBox.frontTopLeft,
		&boundingBox.backTopLeft,
		&boundingBox.backBottomLeft,
		&boundingBox.frontBottomLeft
	) ) return true;

	if ( TestIntersectConvexPolygon(
		&ray,
		4,
		&boundingBox.frontTopLeft,
		&boundingBox.frontTopRight,
		&boundingBox.frontBottomRight,
		&boundingBox.frontBottomLeft
	) ) return true;

	if ( TestIntersectConvexPolygon(
		&ray,
		4,
		&boundingBox.frontTopRight,
		&boundingBox.backTopRight,
		&boundingBox.backBottomRight,
		&boundingBox.frontBottomRight
	) ) return true;

	if ( TestIntersectConvexPolygon(
		&ray,
		4,
		&boundingBox.frontTopLeft,
		&boundingBox.frontTopRight,
		&boundingBox.backTopRight,
		&boundingBox.backTopLeft
	) ) return true;

	if ( TestIntersectConvexPolygon(
		&ray,
		4,
		&boundingBox.frontBottomLeft,
		&boundingBox.frontBottomRight,
		&boundingBox.backBottomRight,
		&boundingBox.backBottomLeft
	) ) return true;

	return false;
}

bool TestIntersectAxisAlignedBox(const Ray& ray, const Box& box)
{
	// algorithm from scratchapixel.com Ray Box Intersection

	float txMin = (box.left - ray.origin.x) / ray.direction.x;
	float txMax = (box.right - ray.origin.x) / ray.direction.x;

	if ( txMin > txMax )
		std::swap(txMin, txMax);

	float tyMin = (box.bottom - ray.origin.y) / ray.direction.y;
	float tyMax = (box.top - ray.origin.y) / ray.direction.y;

	if ( tyMin > tyMax )
		std::swap(tyMin, tyMax);

	if ( txMin > tyMax || tyMin > txMax )
		return false;

	if ( tyMin > txMin )
		txMin = tyMin;
	if ( tyMax < txMax )
		txMax = tyMax;

	float tzMin = (box.back - ray.origin.z) / ray.direction.z;
	float tzMax = (box.front - ray.origin.z) / ray.direction.z;

	if ( tzMin > tzMax )
		std::swap(tzMin, tzMax);

	if ( txMin > tzMax || tzMin > txMax )
		return false;

	return true;
}

bool TestIntersectPlane(const Ray& ray, const Plane& plane)
{
	return TestIntersectConvexPolygon(&ray, 4, &plane.topLeft, &plane.topRight, &plane.bottomRight, &plane.bottomLeft);
}

static void MakeSphere(int vertsDown, float radius, int* numTriangles, int* numVertices, int** indices, Vec4** vertices) {

	// helper function for the sphere constructor

	// the number of vertices around the sphere
	int vertsAround = vertsDown * 2 + 2;

	// the distance in radians between any adjacent vertex
	float radianGap = PI / (vertsDown + 1);

	// number down * number around plus top and bottom
	*numVertices = vertsDown * vertsAround + 2;

	*vertices = new Vec4[*numVertices];

	Vec4 top = { 0, radius, 0, 1 };
	Vec4 bottom = { 0, -radius, 0, 1 };

	// what index into the vertex or index we are
	int count = 0;

	// store top as the first vertex
	*vertices[0] = top;

	// i loops down the sphere
	for (int i = 0; i < vertsDown; ++i) {

		// the z rotation multiplication must be applied first
		float rotz = (i + 1) * radianGap;
		Vec4 down = Mat4::GetRotation(0, 0, rotz) * top;

		// j loops around the sphere
		for (int j = 0; j < vertsAround; ++j) {

			float roty = (j + 1) * radianGap;

			// generate the position
			Vec4 vertex = Mat4::GetRotation(0, roty, 0) * down;

			(*vertices)[1 + count] = vertex;
			++count;
		}
	}

	// store bottom as the last vertex
	(*vertices)[*numVertices - 1] = bottom;

	// number of triangles that connect the top and bottom vertices to the rest of the sphere, excluding
	// the final gap
	int topAndBottom = 2 * (vertsAround - 1);

	// number of triangles that connect the main mesh of the sphere, excluding the final gap
	int mainBody = 2 * (vertsDown - 1) * (vertsAround - 1);

	// number of triangles that connect the final gap, excluding the triangles
	// connecting to the top and bottom
	int finalGap = 2 * (vertsDown - 1);

	// total number of triangles in the sphere is the sum of all of these
	// extra 2 is the two triangles that connect the final gap to the top and botton
	*numTriangles = topAndBottom + mainBody + finalGap + 2;

	// total number of indices is 3 times that
	*indices = new int[*numTriangles * 3];


	count = 0;

	// all indices from here out must add 1 to them to skip over the top index
	// that is first in the list

	// connect to top and bottom (first and last row)
	for (int j = 0; j < vertsAround - 1; ++j) {

		(*indices)[count++] = j + 1;
		(*indices)[count++] = j + 2;
		(*indices)[count++] = 0;

		// vertsdown - 1 is the last row
		(*indices)[count++] = (vertsDown - 1) * vertsAround + j + 1;
		(*indices)[count++] = *numVertices - 1;
		(*indices)[count++] = (vertsDown - 1) * vertsAround + j + 2;
		
	}

	// connect all except last gap
	for (int i = 0; i < vertsDown - 1; ++i) {

		for (int j = 0; j < vertsDown * 2 + 1; ++j) {

			// multiplying a number by vertsAround selects
			// that row

			int i1 = i * vertsAround + j;
			int i2 = (i + 1) * vertsAround + j;
			int i3 = (i + 1) * vertsAround + (j + 1);
			int i4 = i * vertsAround + (j + 1);


			(*indices)[count++] = i1 + 1;
			(*indices)[count++] = i3 + 1;
			(*indices)[count++] = i4 + 1;

			(*indices)[count++] = i1 + 1;
			(*indices)[count++] = i2 + 1;
			(*indices)[count++] = i3 + 1;
		}
	}

	// connect last gap (first column to last column)
	for (int i = 0; i < vertsDown - 1; ++i) {

		int i1 = i * vertsAround;
		int i2 = (i + 1) * vertsAround;
		int i3 = (i + 1) * vertsAround + vertsAround - 1;
		int i4 = i * vertsAround + vertsAround - 1;

		(*indices)[count++] = i1 + 1;
		(*indices)[count++] = i4 + 1;
		(*indices)[count++] = i3 + 1;

		(*indices)[count++] = i1 + 1;
		(*indices)[count++] = i3 + 1;
		(*indices)[count++] = i2 + 1;
		
	}

	// fill in last 2 triangles

	// first row, first and last column
	(*indices)[count++] = 1;
	(*indices)[count++] = 0;
	(*indices)[count++] = (vertsAround - 1) + 1;
	
	// last row, first and last column
	(*indices)[count++] = (vertsDown - 1) * vertsAround + 1;
	(*indices)[count++] = (vertsDown - 1) * vertsAround + (vertsAround - 1) + 1;
	(*indices)[count++] = *numVertices - 1;

}

Sphere::Sphere(int resolution, float radius) : radius(radius) {

	// call make sphere instead of having all that code in this constructor
	MakeSphere(resolution, radius, &nTriangles, &nVertices, &pIndices, &pVertices);

}

Sphere::~Sphere() {

	if (pIndices != nullptr)
		delete[] pIndices;

	if (pVertices != nullptr)
		delete[] pVertices;

}

