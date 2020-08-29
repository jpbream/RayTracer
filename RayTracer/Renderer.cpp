#include "Renderer.h"
#include "Vec2.h"
#include "Mat4.h"
#include <thread>

#define MAX_DIST 1000000000
#define MISS_COLOR Vec4(0, 0, 0, 1);

#define GET_VERTEX(INDEX, VP_VERTS, VSIZE) ((char*)VP_VERTS + (VSIZE * INDEX))
#define GET_POSITION(INDEX, VP_VERTS, VSIZE, VFOFFSET) ((Vec3*)((float*)GET_VERTEX(INDEX, VP_VERTS, VSIZE) + VFOFFSET))

#define MIN_INTERSECTION_DISTANCE 0.00001

#define TREE

Payload::Payload()
	:
	valid(true)
{
}
bool Payload::IsValid() const
{
	return valid;
}

void Renderer::AddModelToScene(void* modelThis, int nTriangles, int* pIndices, int nVertices, void* pVertices, int positionFloatOffset, int vertexSize,
	IntersectBoundingVolume pIntersectBoundingVolume, ClosestHitShader pClosestHit)
{

	ModelDescriptor md = {};
	md.thisPtr = modelThis;
	md.nTriangles = nTriangles;
	md.pIndices = pIndices;
	md.nVertices = nVertices;
	md.pVertices = pVertices;
	md.vertexSize = vertexSize;
	md.positionFloatOffset = positionFloatOffset;
	md.pIntersectBoundingVolume = pIntersectBoundingVolume;
	md.pClosestHitShader = pClosestHit;

	modelStorage.push_back(md);

#ifdef TREE
	ModelDescriptor* model = &modelStorage[modelStorage.size() - 1];
	octTree.AddModelToTree(model);
#endif

}

void Renderer::RenderScene(Surface* pRenderTarget, RayGenerationShader pRayGen, MissShader pMissShader)
{

	if ( pRenderTarget == nullptr || pRayGen == nullptr || pMissShader == nullptr )
		return;
	if ( pRenderTarget->GetWidth() == 0 || pRenderTarget->GetHeight() == 0 )
		return;

	this->pRayGen = pRayGen;
	this->pMissShader = pMissShader;
	this->pRenderTarget = pRenderTarget;

	static int numThreads = std::thread::hardware_concurrency() - 1;
	int numPixels = pRenderTarget->GetWidth() * pRenderTarget->GetHeight();

	// in case the image has less than 12 pixels
	if ( numPixels < numThreads )
		return;

	int span = numPixels / numThreads;
	int newThreadEnd = span * numThreads;

	std::vector<std::thread> threads;
	for ( int start = 0; start < newThreadEnd; start += span )
		threads.emplace_back(&Renderer::RenderThread, this, start, span);

	RenderThread(newThreadEnd, numPixels - newThreadEnd);

	for ( std::thread& thread : threads )
		thread.join();

	this->pRayGen = nullptr;
	this->pMissShader = nullptr;
	this->pRenderTarget = nullptr;

}

void Renderer::RenderThread(int start, int span)
{

	int width = pRenderTarget->GetWidth();
	int height = pRenderTarget->GetHeight();

	for ( int p = start; p < start + span; ++p ) {

		// get the pixel coordinates on the screen
		int px = p % width;
		int py = p / width;

		Vec2 center((float)px + 0.5f * RESOLUTION, (float)py + 0.5f * RESOLUTION);

		float centerXStart = center.x;
		float centerInc = 1.0f / RESOLUTION;
	
		Vec3 accumAvg;

		for ( int i = 0; i < RESOLUTION; ++i ) {
			for ( int j = 0; j < RESOLUTION; ++j ) {

				Ray ray = pRayGen(center.x, center.y, width, height);
				RayTracer rayTracer(this);
				Payload result = TraceRay(ray, rayTracer);

				accumAvg += result.color;
				center.x += centerInc;
			}

			center.x = centerXStart;
			center.y += centerInc;
		}

		accumAvg /= (RESOLUTION * RESOLUTION);

		pRenderTarget->PutPixel(px, py, accumAvg);
	}

}

Renderer::RayTracer::RayTracer(Renderer* renderer)
	:
	renderer(renderer)
{
	traceCount = 0;
}

Payload Renderer::RayTracer::TraceRay(const Ray& ray)
{
	Payload payload;

	if ( traceCount == MAX_TRACE ) {

		// return an invalid payload
		payload.valid = false;
		return payload;
	}

	traceCount++;
	payload = renderer->TraceRay(ray, *this);
	traceCount--;

	return payload;

}

Payload Renderer::TraceRay(const Ray& ray, RayTracer& rayTracer)
{

	// only keeping track of the closest hit, so it is not possible
	// to ignore hits
	float minDist = MAX_DIST;
	ModelDescriptor* closestHit = nullptr;
	TriangleIntersection closestIntersection;

#ifndef TREE
	for ( ModelDescriptor& md : modelStorage ) {

		if ( md.pIntersectBoundingVolume(md.thisPtr, ray) ) {

			// loop through all triangles in the model
			for ( int i = 0; i < md.nTriangles * 3; i += 3 ) {

				// get the three positions of the vertices using a bunch of pointer trickery
				Vec3* vertex1 = GET_POSITION(md.pIndices[i], md.pVertices, md.vertexSize, md.positionFloatOffset);
				Vec3* vertex2 = GET_POSITION(md.pIndices[i + 1], md.pVertices, md.vertexSize, md.positionFloatOffset);
				Vec3* vertex3 = GET_POSITION(md.pIndices[i + 2], md.pVertices, md.vertexSize, md.positionFloatOffset);

				TriangleIntersection intersection;
				if ( IntersectTriangle(ray, *vertex1, *vertex2, *vertex3, intersection) ) {

					intersection.triangleIdx = i / 3;

					// this is where the any hit shader would be run

					if ( intersection.distance < minDist ) {

						// found a new closest hit
						minDist = intersection.distance;
						closestHit = &md;
						closestIntersection = intersection;

					}
				}
			}
		}
	}

	Payload payload;
	if ( closestHit ) {
		payload = closestHit->pClosestHitShader(closestHit->thisPtr, rayTracer, ray, closestIntersection);
	}
	else {
		payload = pMissShader(ray);
	}

	return payload;

#else
	// run the closest hit shader of the nearest object
	// if there is no nearest object, run the ray tracers miss shader

	Payload payload;
	if ( octTree.IntersectRayWithTree(ray, closestHit, closestIntersection) ) {
		payload = closestHit->pClosestHitShader(closestHit->thisPtr, rayTracer, ray, closestIntersection);
	}
	else {
		payload = pMissShader(ray);
	}

	return payload;
#endif
}

Renderer::SceneOctTree::SceneOctTree()
{

	static const Box sceneBoundingBox = {-20, 20, -20, 20, -20, 20};

	root = new SceneOctTreeNode(sceneBoundingBox, 1);
}
Renderer::SceneOctTree::~SceneOctTree()
{
	delete root;
}
Renderer::SceneOctTree::SceneOctTreeNode::SceneOctTreeNode(const Box& box, int level)
	:
	box(box)
{
	if ( level == DEPTH ) {
		isLeaf = true;
	}
	else {

		isLeaf = false;

		float midY = (box.bottom + box.top) / 2;
		float midX = (box.left + box.right) / 2;
		float midZ = (box.back + box.front) / 2;

		Box backTopLeft = { box.left, midX, midY, box.top, box.back, midZ };
		Box backTopRight = { midX, box.right, midY, box.top, box.back, midZ };
		Box backBottomLeft = { box.left, midX, box.bottom, midY, box.back, midZ };
		Box backBottomRight = { midX, box.right, box.bottom, midY, box.back, midZ };

		Box frontTopLeft = { box.left, midX, midY, box.top, midZ, box.front };
		Box frontTopRight = { midX, box.right, midY, box.top, midZ, box.front };
		Box frontBottomLeft = { box.left, midX, box.bottom, midY, midZ, box.front };
		Box frontBottomRight = { midX, box.right, box.bottom, midY, midZ, box.front };

		children[0] = new SceneOctTreeNode(backTopLeft, level + 1);
		children[1] = new SceneOctTreeNode(backTopRight, level + 1);
		children[2] = new SceneOctTreeNode(backBottomLeft, level + 1);
		children[3] = new SceneOctTreeNode(backBottomRight, level + 1);
		children[4] = new SceneOctTreeNode(frontTopLeft, level + 1);
		children[5] = new SceneOctTreeNode(frontTopRight, level + 1);
		children[6] = new SceneOctTreeNode(frontBottomLeft, level + 1);
		children[7] = new SceneOctTreeNode(frontBottomRight, level + 1);
	}
}
Renderer::SceneOctTree::SceneOctTreeNode::~SceneOctTreeNode()
{
	if ( !isLeaf ) {
		delete children[0];
		delete children[1];
		delete children[2];
		delete children[3];
		delete children[4];
		delete children[5];
		delete children[6];
		delete children[7];
	}
}
Renderer::SceneOctTree::SceneOctTreeNode* Renderer::SceneOctTree::FindBoxContainingTriangle(const Triangle& t)
{
	// test triangles against whole scene (root)
	if ( !TestTriangleInsideBox(t, root->box) )
		return nullptr;

	SceneOctTreeNode* node = root;
	SceneOctTreeNode* prevNode = nullptr;

	while ( prevNode != node && !node->isLeaf) {

		prevNode = node;

		// update node to be its child box that contains the triangle
		for (int i = 0; i < 8; ++i)
			if ( TestTriangleInsideBox(t, node->children[i]->box) ) {
				node = node->children[i];
				break;
			}

		// if node did not change, node is the smallest box that
		// contains the triangle

	}
	
	return node;
}

void Renderer::SceneOctTree::AddModelToTree(Renderer::ModelDescriptor* model)
{
	for ( int i = 0; i < model->nTriangles * 3; i += 3 ) {

		const Vec3& v1 = *GET_POSITION(model->pIndices[i], model->pVertices, model->vertexSize, model->positionFloatOffset);
		const Vec3& v2 = *GET_POSITION(model->pIndices[i + 1], model->pVertices, model->vertexSize, model->positionFloatOffset);
		const Vec3& v3 = *GET_POSITION(model->pIndices[i + 2], model->pVertices, model->vertexSize, model->positionFloatOffset);

		Triangle t = { v1, v2, v3 };

		// would do backface culling here

		SceneOctTreeNode* box = FindBoxContainingTriangle(t);
		if ( box )
			box->trianglesInBox[model].push_back((std::make_tuple(i / 3, &v1, &v2, &v3)));

	}
}

bool Renderer::SceneOctTree::SceneOctTreeNode::IntersectRay(const Ray& ray, ModelDescriptor*& outModel, TriangleIntersection& outIntersection) const
{

	// check if the ray intersects this box
	if ( TestIntersectAxisAlignedBox(ray, box) ) {

		float minDist = MAX_DIST;
		ModelDescriptor* closestModel = nullptr;
		TriangleIntersection closestIntersection;

		TriangleIntersection currentIntersection;

		// loop through all models with triangles in this box
		for ( auto it : trianglesInBox ) {

			ModelDescriptor* model = it.first;
			std::vector<TriangleDesc>& triangles = it.second;

			// loop through all triangles in that model
			for ( TriangleDesc& triangle : triangles ) {
				if ( IntersectTriangle(ray, *std::get<1>(triangle), *std::get<2>(triangle), *std::get<3>(triangle), currentIntersection) 
					&& currentIntersection.distance < minDist) 
				{
					minDist = currentIntersection.distance;
					closestIntersection = currentIntersection;

					// remember to record the triangle index and model
					closestModel = model;
					closestIntersection.triangleIdx = std::get<0>(triangle);
				}
			}
		}

		// if this node is not a leaf, check its children for intersections
		if ( !isLeaf ) {

			ModelDescriptor* childModel = nullptr;
			for ( int i = 0; i < 8; ++i ) {

				if ( children[i]->IntersectRay(ray, childModel, currentIntersection) && currentIntersection.distance < minDist ) {

					// this child has reported an intersection closer than the closest known one
					minDist = currentIntersection.distance;
					closestIntersection = currentIntersection;
					closestModel = childModel;
				}
			}
		}

		if ( closestModel )
			outModel = closestModel;
		else
			return false;

		outIntersection = closestIntersection;
		return true;
	}

	return false;
}

bool Renderer::SceneOctTree::IntersectRayWithTree(const Ray& ray, Renderer::ModelDescriptor*& outModel, TriangleIntersection& outIntersection) const
{
	return root->IntersectRay(ray, outModel, outIntersection);
}

bool Renderer::IntersectTriangle(const Ray& ray, const Vec3& v1, const Vec3& v2, const Vec3& v3, TriangleIntersection& outIntersection)
{

	// specialized version of the intersect triangle from shapes.h

	Vec3 side1 = v2 - v1;
	Vec3 side2 = v3 - v1;

	Vec3 norm = (side1 % side2);
	float triangleArea = norm.Length() / 2;
	norm = norm.Normalized();

	// the distance along the planes normal the triangle
	// is from the origin
	float d = norm * v1;

	// if the ray is perpendicular to the normal,
	// ignore this triangle
	if ( norm * ray.direction == 0 )
		return false;

	// the distance from the rays origin to its intersection
	// with the plane
	float t = (norm * ray.origin + d) / (norm * ray.direction);

	// if the triangle is behind or is the starting point of the ray, ignore it
	if ( t < MIN_INTERSECTION_DISTANCE )
		return false;

	// calculate the point of intersection with the plane
	Vec3 intersection = ray.origin + ray.direction * t;

	// perform the inside outside test
	// if the intersection point is outside the triangle,
	// it will be ignored
	
	Vec3 edge0 = v2 - v1;
	Vec3 c0 = intersection - v1;
	Vec3 cp0 = edge0 % c0;
	if ( norm * cp0 <= 0 )
		return false;

	Vec3 edge1 = v3 - v2;
	Vec3 c1 = intersection - v2;
	Vec3 cp1 = edge1 % c1;
	if ( norm * cp1 <= 0 )
		return false;

	Vec3 edge2 = v1 - v3;
	Vec3 c2 = intersection - v3;
	Vec3 cp2 = edge2 % c2;
	if ( norm * cp2 <= 0 )
		return false;
	
	// calculate barycentric coordinates of v2 and v3
	// coordinate of v1 is 1 - u - v
	float u = cp1.Length() / 2 / triangleArea;
	float v = cp2.Length() / 2 / triangleArea;
	
	outIntersection.distance = t;
	if ( norm * ray.direction > 0 )
		outIntersection.fromBack = true;
	else
		outIntersection.fromBack = false;

	outIntersection.u = u;
	outIntersection.v = v;

	return true;
}

void Renderer::ClearScene()
{
	octTree.ClearTree();
	modelStorage.clear();
}
void Renderer::SceneOctTree::ClearTree()
{
	root->Clear();
}
void Renderer::SceneOctTree::SceneOctTreeNode::Clear()
{
	trianglesInBox.clear();

	if ( !isLeaf ) {
		children[0]->Clear();
		children[1]->Clear();
		children[2]->Clear();
		children[3]->Clear();
		children[4]->Clear();
		children[5]->Clear();
		children[6]->Clear();
		children[7]->Clear();
	}
}