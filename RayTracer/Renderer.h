#pragma once
#include "Surface.h"
#include "Vec3.h"
#include "Shapes.h"
#include <vector>
#include <unordered_map>

#define RESOLUTION 1
#define BOUNDING_BOX_TEST
#define THREAD_SWITCHING

class Payload;

struct TriangleIntersection {

	int triangleIdx;

	float u;
	float v;

	float distance;
};

class Renderer
{
	friend class RayTracer;
public:

	class RayTracer {
		friend class Renderer;

	private:
		int traceCount;
		static const int MAX_TRACE = 5;

		Renderer* renderer;

	public:
		RayTracer(Renderer* renderer);
		Payload TraceRay(const Ray& ray);

		int RecursionLevel() const;
	};

	typedef Ray (*RayGenerationShader)(float px, float py, int displayWidth, int displayHeight);
	typedef Payload (*ClosestHitShader)(void* thisPtr, Renderer::RayTracer& rayTracer, const Ray& ray, const TriangleIntersection& intersection);
	typedef Payload (*MissShader)(const Ray& ray);

	typedef bool (*BoundingVolumeTest)(void* thisPtr, const Ray& ray);

private:
	MissShader pMissShader;
	RayGenerationShader pRayGen;
	Surface* pRenderTarget;

	class ModelDescriptor {	
		// use this to get the default copy constructor
		friend class Renderer;

		void* thisPtr;

		int nTriangles;
		int nVertices;

		int* pIndices;
		void* pVertices;
		int vertexSize;
		int positionFloatOffset;

		ClosestHitShader pClosestHitShader;
		BoundingVolumeTest pBoundingVolumeTest;

		bool backfaceCull;
	};

	class SceneOctTree {
	private:
		static constexpr int DEPTH = 6;

		class SceneOctTreeNode {
			typedef std::tuple<int, const Vec3*, const Vec3*, const Vec3*> TriangleDesc;

		public:
			SceneOctTreeNode* children[8];
			Box box;

			bool isLeaf;
			std::unordered_map<ModelDescriptor*, std::vector<TriangleDesc>> trianglesInBox;

			SceneOctTreeNode(const Box& box, int level);
			~SceneOctTreeNode();

			bool IntersectRay(const Ray& ray, ModelDescriptor*& outModel, TriangleIntersection& outIntersection) const;
			void Clear();

		};

		SceneOctTreeNode* root;

		SceneOctTreeNode* FindBoxContainingTriangle(const Triangle& t);

	public:
		SceneOctTree();
		~SceneOctTree();

		void AddModelToTree(ModelDescriptor* model);
		void ClearTree();
		bool IntersectRayWithTree(const Ray& ray, ModelDescriptor*& outModel, TriangleIntersection& outIntersection) const;

	};

	SceneOctTree octTree;
	std::vector<ModelDescriptor> modelStorage;

	struct WorkerRange {
		volatile int start;
		volatile int end;
	};

	std::vector<WorkerRange> workers;

	void RenderThread(int threadIdx);
	Payload TraceRay(const Ray& ray, RayTracer& rayTracer);

	static bool IntersectTriangle(const Ray& ray, const Vec3& v1, const Vec3& v2, const Vec3& v3, TriangleIntersection& outIntersection);

public:

	void AddModelToScene(void* modelThis, int nTriangles, int* pIndices, int nVertices, void* pVertices, int positionFloatOffset, int vertexSize,
		 ClosestHitShader pClosestHit, BoundingVolumeTest pBoundingVolumeTest, bool backfaceCull);

	void ClearScene();

	void RenderScene(Surface* pRenderTarget, RayGenerationShader pRayGen, MissShader pMissShader);

};

struct Payload {

	Vec3 color;

};

