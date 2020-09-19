#include "Window.h"
#include "Vec2.h"
#include "Mat4.h"
#include "Mat3.h"
#include "Shapes.h"
#include "Importing.h"
#include "Utility.h"
#include "Sampling.h"
#include "Renderer.h"
#include "Lighting.h"

#include <iostream>
#include <math.h>
#include <thread>
#include <vector>

#define WIDTH 1920
#define HEIGHT 1080

Vec3 light(0, 0, -1);

bool IntersectSphere(void* thisPtr, const Ray& ray);

bool BoundingTest(void* thisPtr, const Ray& ray);

Payload ClosestHit(void* thisPtr, Renderer::RayTracer& rayTracer, const Ray& ray, const TriangleIntersection& intersection);

class CowModel {

	friend Payload ClosestHit(void* thisPtr, Renderer::RayTracer& rayTracer, const Ray& ray, const TriangleIntersection& intersection);
	friend bool BoundingTest(void* thisPtr, const Ray& ray);

private:

	int nVertices;
	int nTriangles;

	int* pIndices;

	Surface normalMap;

	class Vertex {
	public:
		Vec4 position;
		Vec3 worldPos;
		Vec3 normal;
		Vec2 texel;
		Vec3 tangent;
		Vec3 bitangent;

		float padding[6];
	};

	BoundingBox bb;

	Vertex* pVertices;

	Mat4 rot;
	Mat4 move;
	Mat4 scale;

public:

	CowModel(const Vec3& pos)
		:
		normalMap("images/norm.png")
	{
		rot = Mat4::GetRotation(0, 0, 0);
		move = Mat4::Get3DTranslation(pos.x, pos.y, pos.z);
		scale = Mat4::GetScale(1, 1, 1);

		Scene scene("models/OBJ/Cow2.obj");
		Mesh cow = scene.MeshAt(0);

		nVertices = cow.NumVertices();
		pVertices = new Vertex[nVertices];

		nTriangles = cow.NumTriangles();
		pIndices = new int[nTriangles * 3];

		for ( int i = 0; i < nVertices; ++i ) {
			pVertices[i].position = cow.Positions(i).Vec4();
			pVertices[i].normal = cow.Normals(i);
		}

		for ( int i = 0; i < nTriangles * 3; ++i ) {
			pIndices[i] = cow.Indices(i);
		}

		/*nVertices = 4;
		pVertices = new Vertex[nVertices];
		pVertices[0].position = { -1, 1, 0, 1 };
		pVertices[0].normal = { 0, 0, 1 };
		pVertices[0].texel = { 0, 0 };

		pVertices[1].position = { -1, -1, 0, 1 };
		pVertices[1].normal = { 0, 0, 1 };
		pVertices[1].texel = { 0, 1 };

		pVertices[2].position = { 1, -1, 0, 1 };
		pVertices[2].normal = { 0, 0, 1 };
		pVertices[2].texel = { 1, 1 };

		pVertices[3].position = { 1, 1, 0, 1 };
		pVertices[3].normal = { 0, 0, 1 };
		pVertices[3].texel = { 1, 0 };

		nTriangles = 2;
		pIndices = new int[6];
		pIndices[0] = 0;
		pIndices[1] = 1;
		pIndices[2] = 3;
		pIndices[3] = 1;
		pIndices[4] = 2;
		pIndices[5] = 3;

		CalculateTangentsAndBitangents(nTriangles, pIndices, nVertices, pVertices,
			FLOAT_OFFSET(pVertices[0], position),
			FLOAT_OFFSET(pVertices[0], texel),
			FLOAT_OFFSET(pVertices[0], normal),
			FLOAT_OFFSET(pVertices[0], tangent),
			FLOAT_OFFSET(pVertices[0], bitangent)
		);
		scale = Mat4::GetScale(5, 5, 5);*/
		

		bb = GenerateBoundingBox(nVertices, pVertices, FLOAT_OFFSET(pVertices[0], position));

		for ( int i = 0; i < nVertices; ++i ) {
			pVertices[i].worldPos = (move * rot * scale * pVertices[i].position).Vec3();
			pVertices[i].normal = (rot * pVertices[i].normal.Vec4()).Vec3();
		}
		for ( int i = 0; i < 8; ++i ) {
			Vec3& corner = *((Vec3*)&bb + i);
			corner = (move * rot * scale * corner.Vec4()).Vec3();
		}

	}

	~CowModel()
	{
		delete[] pIndices;
		delete[] pVertices;
	}

	void AddToScene(Renderer& r)
	{
		r.AddModelToScene(this, nTriangles, pIndices, nVertices, pVertices, FLOAT_OFFSET(pVertices[0], worldPos), sizeof(Vertex), ClosestHit, BoundingTest, false);
	}
};

bool BoundingTest(void* thisPtr, const Ray& ray)
{
	return TestIntersectBox(ray, ((CowModel*)thisPtr)->bb);
}

Payload ClosestHit(void* thisPtr, Renderer::RayTracer& rayTracer, const Ray& ray, const TriangleIntersection& intersection)
{
	Payload payload;
	CowModel* cow = (CowModel*)thisPtr;

	int i1 = cow->pIndices[intersection.triangleIdx * 3];
	int i2 = cow->pIndices[intersection.triangleIdx * 3 + 1];
	int i3 = cow->pIndices[intersection.triangleIdx * 3 + 2];

	CowModel::Vertex& v1 = cow->pVertices[i1];
	CowModel::Vertex& v2 = cow->pVertices[i2];
	CowModel::Vertex& v3 = cow->pVertices[i3];
	
	CowModel::Vertex hit = BarycentricLerp(v1, v2, v3, intersection.u, intersection.v);
	hit.normal = hit.normal.Normalized();

	/*Vec3 realNormal = SampleNormalMap(cow->normalMap, hit.texel);
	Mat3 tanToObj(hit.tangent, hit.bitangent, hit.normal.Normalized());
	Vec3 n = tanToObj * realNormal;
	n = (cow->rot * n.Vec4()).Vec3();
	hit.normal = n;*/

	Vec3 toLight = (light - hit.worldPos).Normalized();
	Vec3 lightCol = { 0.5, 0.5, 0.5 };
	Vec3 toCam = (Vec3{ 0, 0, 0 } - hit.worldPos).Normalized();

	//Ray shadow;
	//shadow.direction = -light.Normalized();
	//shadow.origin = hit.worldPos;
	//Payload shadowHit = rayTracer.TraceRay(shadow);

	// how much the surface faces the light
	float facingFactor = FacingFactor(light, hit.normal);

	// spec factor is how much to scale the specular color by
	float specFactor = SpecularFactor(toLight, hit.normal, toCam, 30);

	//if ( shadowHit.intersected )
		//specFactor = 0;

	Ray reflection;
	reflection.origin = hit.worldPos;
	reflection.direction = (-ray.direction).Reflect(hit.normal);
	Payload refl = rayTracer.TraceRay(reflection);

	// the colors based on the materials surface properties
	// diffuse, specular (specular color will be the light color)
	Vec3 nonLightCol = Vec3(0, 0, 1) * facingFactor + lightCol * specFactor + refl.color;

	// final non ambient color is the color of the light modulated with
	// the colors not contributed by the light
	Vec3 nonAmbientColor = Vec3::Modulate(
		lightCol,
		nonLightCol
	);

	// ambient and emmissive color would be added to this
	Vec3 finalColor = nonAmbientColor;
	//if ( shadowHit.intersected )
		//finalColor -= {0.3, 0.3, 0.3};

	finalColor.Clamp();
	payload.color = finalColor;
	payload.intersected = true;
	return payload;

}

float fov = 90;
Ray PinholeCameraRayGeneration(float px, float py, int displayWidth, int displayHeight)
{

	// calculate the camera space to world space matrix
	//Mat4 camSpaceMatrix = Mat4::Get3DTranslation(pCamera->position.x, pCamera->position.y, pCamera->position.z) *
	//Mat4::GetRotation(pCamera->rotation.x, pCamera->rotation.y, pCamera->rotation.z);

	// use the cameras origin as the rays origin
	Vec3 origin(0, 0, 0);
	//origin = (camSpaceMatrix * origin.Vec4()).Vec3();

	// in ray tracing, ndc space is from [0, 1]
	Vec2 ndc(px / (float)displayWidth, py / (float)displayHeight);

	// in ray tracing, screen space is [-1, 1]
	Vec2 screen(2 * ndc.x - 1, 1 - 2 * ndc.y);

	// scale x by aspect ratio
	screen.x *= (float)displayWidth / displayHeight;

	// scale screen by the field of view
	screen *= tan((fov / 2) * (PI / 180));

	// screen point is the pixels point in camera space,
	// give a z value of -1
	Vec3 camSpace(screen.x, screen.y, -1);
	//camSpace = (camSpaceMatrix * camSpace.Vec4()).Vec3();

	// the rays direction is its point on the cameras viewing plane
	// minus the cameras origin
	Vec3 dir = (camSpace - origin).Normalized();

	Ray ray = { origin, dir };
	
	return ray;
}

Surface cubeMap[] = {
	Surface("cube/posx.jpg"),
	Surface("cube/negx.jpg"),
	Surface("cube/posy.jpg"),
	Surface("cube/negy.jpg"),
	Surface("cube/posz.jpg"),
	Surface("cube/negz.jpg")
};


Payload Miss(const Ray& ray)
{
	Payload payload;
	payload.color = SampleCubeMap(cubeMap, ray.direction.Normalized()).Vec3();

	payload.intersected = false;

	return payload;
}

Window wnd("My Window", 12, 12, WIDTH, HEIGHT, 0);
Surface surf(WIDTH, HEIGHT);

volatile bool shouldQuit = false;

void DrawLoop()
{
	while ( !shouldQuit ) {
		wnd.DrawSurface(surf);
	}
}

int main(int argc, char* argv[])
{
	std::thread t(DrawLoop);

	Renderer renderer;
	CowModel cm({ 0, 0, -9 });
	CowModel cm2({ -5, 0, -5 });
	
	
	double start = (double)SDL_GetPerformanceCounter() / SDL_GetPerformanceFrequency();

	cm.AddToScene(renderer);
	cm2.AddToScene(renderer);
	renderer.RenderScene(&surf, PinholeCameraRayGeneration, Miss);
	renderer.ClearScene();
	
	double end = (double)SDL_GetPerformanceCounter() / SDL_GetPerformanceFrequency();
	std::cout << (end - start) << " seconds" << std::endl;

	//wnd.DrawSurface(surf);
	wnd.BlockUntilQuit();

	shouldQuit = true;
	t.join();
	return 0;

}