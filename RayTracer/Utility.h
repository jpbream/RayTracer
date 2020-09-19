#pragma once
#include "Mat2.h"
#include "Vec3.h"

#include <immintrin.h>

#define FLOAT_OFFSET(OBJECT, MEMBER) (int)((float*)&OBJECT.MEMBER - (float*)&OBJECT)

inline float HAdd4(__m128 sse)
{
	// compute horizontal sum of the 128 bit vector

	// get the lower 2 floats by casting (this is just for show)
	__m128 lowTwo = sse;

	// move the upper 2 into the lower 2
	__m128 highTwo = _mm_movehl_ps(sse, sse);

	// add the high's and low's
	__m128 sumTwo = _mm_add_ps(lowTwo, highTwo);

	// return the sum of the two remaining values
	return sumTwo.m128_f32[0] + sumTwo.m128_f32[1];
}

inline float HAdd8(__m256 avx)
{

	// compute horizontal sum of the 256 bit vector

	// extract upper 4 floats into a 128
	__m128 highQuad = _mm256_extractf128_ps(avx, 1);

	// get lower 4 floats into a 128 by casting
	__m128 lowQuad = _mm256_castps256_ps128(avx);

	// add the high's and low's
	__m128 sumQuad = _mm_add_ps(highQuad, lowQuad);

	// get the lower 2 floats by casting (this is just for show)
	__m128 lowTwo = sumQuad;

	// move the upper 2 into the lower 2
	__m128 highTwo = _mm_movehl_ps(sumQuad, sumQuad);

	// add the high's and low's
	__m128 sumTwo = _mm_add_ps(lowTwo, highTwo);

	// return the sum of the two remaining values
	return sumTwo.m128_f32[0] + sumTwo.m128_f32[1];

}

template <class FloatType>
FloatType BarycentricLerp(const FloatType& p1, const FloatType& p2, const FloatType& p3, float u, float v)
{

	constexpr short NUMFLOATS = sizeof(FloatType) / sizeof(float);
	float alignas(32) buf[NUMFLOATS];

	if constexpr ( NUMFLOATS % 8 == 0 ) {

		__m256 vecU = _mm256_set1_ps(u);
		__m256 vecV = _mm256_set1_ps(v);
		__m256 vecW = _mm256_set1_ps(1 - u - v);

		for ( int i = 0; i < NUMFLOATS / 8; ++i ) {

			__m256 vec1 = _mm256_load_ps((float*)&p1 + i * 8);
			__m256 vec2 = _mm256_load_ps((float*)&p2 + i * 8);
			__m256 vec3 = _mm256_load_ps((float*)&p3 + i * 8);

			vec1 = _mm256_mul_ps(vec1, vecU);
			vec2 = _mm256_mul_ps(vec2, vecV);
			vec3 = _mm256_mul_ps(vec3, vecW);

			__m256 result = _mm256_add_ps(vec1, vec2);
			result = _mm256_add_ps(result, vec3);

			_mm256_store_ps(buf + i * 8, result);
		}
	}
	else {

		float w = 1 - u - v;

		for ( int i = 0; i < NUMFLOATS; ++i )
			buf[i] = *((float*)&p1 + i) * u + *((float*)&p2 + i) * v + *((float*)&p3 + i) * w;
	}

	return *(FloatType*)buf;

}

template <class FloatType>
FloatType Lerp(const FloatType& p1, const FloatType& p2, float alpha)
{

	// do not use this for small objects, it will be outperformed by the operator overloads

	constexpr short NUMFLOATS = sizeof(FloatType) / sizeof(float);
	float alignas(32) buf[NUMFLOATS];
	
	if constexpr ( NUMFLOATS % 8 == 0 ) {
		
		__m256 first = _mm256_set1_ps(1 - alpha);
		__m256 second = _mm256_set1_ps(alpha);

		for ( int i = 0; i < NUMFLOATS / 8; ++i ) {

			__m256 vec1 = _mm256_load_ps((float*)&p1 + i * 8);
			__m256 vec2 = _mm256_load_ps((float*)&p2 + i * 8);

			vec1 = _mm256_mul_ps(vec1, first);
			vec2 = _mm256_mul_ps(vec2, second);

			__m256 result = _mm256_add_ps(vec1, vec2);

			_mm256_store_ps(buf + i * 8, result);
		}
	}
	else {

		for ( int i = 0; i < NUMFLOATS; ++i )
			buf[i] = *((float*)&p1 + i) * (1 - alpha) + *((float*)&p2 + i) * alpha;
	}

	return *(FloatType*)buf;
}

template <class FloatType>
FloatType Mult(const FloatType& p, float s)
{

	// do not use this for small objects, it will be outperformed by the operator overloads

	constexpr short NUMFLOATS = sizeof(FloatType) / sizeof(float);
	float alignas(32) buf[NUMFLOATS];

	if constexpr ( NUMFLOATS % 8 == 0 ) {

		__m256 sVec = _mm256_set1_ps(s);

		for ( int i = 0; i < NUMFLOATS / 8; ++i ) {

			__m256 vec = _mm256_load_ps((float*)&p + i * 8);
			__m256 result = _mm256_mul_ps(vec, sVec);

			_mm256_store_ps(buf + i * 8, result);
		}

	}
	else {

		for ( int i = 0; i < NUMFLOATS; ++i )
			buf[i] = *((float*)&p + i) * s;
	}

	return *(FloatType*)buf;

}

template <class Vertex>
void CalculateNormals(
	int numTriangles, 
	const int* pIndices, 
	int numVertices, 
	Vertex* pVertices, 
	int positionOffset, 
	int outputNormalOffset
)
{

	// will accumulate the unnormalized normal vectors for each vertex
	Vec3* normals = new Vec3[numVertices];

	memset((void*)normals, 0, sizeof(Vec3) * numVertices);

	// loop through each triangle
	for ( int i = 0; i < numTriangles; ++i ) {

		// get indices for this triangle
		int idx1 = pIndices[i * 3];
		int idx2 = pIndices[i * 3 + 1];
		int idx3 = pIndices[i * 3 + 2];

		// get positions for this triangle from the vertices
		const Vec3& p1 = *(Vec3*)((float*)&pVertices[idx1] + positionOffset);
		const Vec3& p2 = *(Vec3*)((float*)&pVertices[idx2] + positionOffset);
		const Vec3& p3 = *(Vec3*)((float*)&pVertices[idx3] + positionOffset);

		// find face normal using cross product of the sides
		Vec3 faceNormal = (p2 - p1) % (p3 - p1);

		// update the accumulators for each vertex on this triangle
		normals[idx1] += faceNormal;
		normals[idx2] += faceNormal;
		normals[idx3] += faceNormal;

	}

	// loop through each vertex and store the normalized normal
	for ( int i = 0; i < numVertices; ++i )
		*(Vec3*)((float*)(&pVertices[i]) + outputNormalOffset) = normals[i].Normalized();
	

	delete[] normals;

}

template <class Vertex>
void CalculateTangentsAndBitangents(
	int numTriangles, 
	const int* pIndices,
	int numVertices,
	Vertex* pVertices, 
	int positionOffset, 
	int texCoordOffset, 
	int normalOffset, 
	int outputTangentOffset, 
	int outputBitangentOffset
)
{
	// Tangent and Bitangent calculation method from
	// "Mathematics for 3D Game Programming and Computer Graphics"
	// by Eric Lengyel, section 7.8

	// will accumulate the unnormalized tangents and bitangents for each vertex
	Vec3* tangents = new Vec3[numVertices];
	Vec3* bitangents = new Vec3[numVertices];

	memset((void*)tangents, 0, sizeof(Vec3) * numVertices);
	memset((void*)bitangents, 0, sizeof(Vec3) * numVertices);

	// loop through each triangle
	for ( int i = 0; i < numTriangles; ++i ) {

		// get indices for this triangle
		int idx1 = pIndices[i * 3];
		int idx2 = pIndices[i * 3 + 1];
		int idx3 = pIndices[i * 3 + 2];

		// get positions for this triangle from the vertices
		const Vec3& p1 = *(Vec3*)((float*)&pVertices[idx1] + positionOffset);
		const Vec3& p2 = *(Vec3*)((float*)&pVertices[idx2] + positionOffset);
		const Vec3& p3 = *(Vec3*)((float*)&pVertices[idx3] + positionOffset);

		// get normal map coords for this triangle from the vertices
		const Vec2& t1 = *(Vec2*)((float*)&pVertices[idx1] + texCoordOffset);
		const Vec2& t2 = *(Vec2*)((float*)&pVertices[idx2] + texCoordOffset);
		const Vec2& t3 = *(Vec2*)((float*)&pVertices[idx3] + texCoordOffset);

		// position and texture coord vectors of the sides of the triangle
		Vec3 side1 = p2 - p1;
		Vec3 side2 = p3 - p1;

		Vec2 tex1 = t2 - t1;
		Vec2 tex2 = t3 - t1;

		// next steps are to solve the matrix equation
		// [ Side1 ] = [ tex1 ] * [  Tangent  ]
		// [ Side2 ]   [ tex2 ]   [ Bitangent ]
		//
		// for Tangent and Bitangent

		Mat2 texMat (
			{ tex1.s, tex2.s },
			{ tex1.t, tex2.t }
		);
		texMat = texMat.GetInverse();

		// have to do it this way since my matrix library
		// only supports square matrices

		Vec3 tangent, bitangent;

		tangent.x = texMat(0, 0) * side1.x + texMat(0, 1) * side2.x;
		tangent.y = texMat(0, 0) * side1.y + texMat(0, 1) * side2.y;
		tangent.z = texMat(0, 0) * side1.z + texMat(0, 1) * side2.z;

		bitangent.x = texMat(1, 0) * side1.x + texMat(1, 1) * side2.x;
		bitangent.y = texMat(1, 0) * side1.y + texMat(1, 1) * side2.y;
		bitangent.z = texMat(1, 0) * side1.z + texMat(1, 1) * side2.z;

		// update the accumulators for each vertex on this triangle
		tangents[idx1] += tangent;
		tangents[idx2] += tangent;
		tangents[idx3] += tangent;

		bitangents[idx1] += bitangent;
		bitangents[idx2] += bitangent;
		bitangents[idx3] += bitangent;
	}

	// loop through each vertex
	for ( int i = 0; i < numVertices; ++i ) {

		const Vec3& normal = *(Vec3*)((float*)&pVertices[i] + normalOffset);
		const Vec3& tangent = tangents[i];
		const Vec3& bitangent = bitangents[i];

		// use Graham Schmidt Orthogonalization on the vectors since they might not
		// be completely orthogonal

		Vec3 finalTangent = tangent - normal * (normal * tangent);
		Vec3 finalBitangent = bitangent - normal * (normal * bitangent) - finalTangent * (finalTangent * bitangent);

		// finally time to normalize the vectors
		*(Vec3*)((float*)&pVertices[i] + outputTangentOffset) = finalTangent.Normalized();
		*(Vec3*)((float*)&pVertices[i] + outputBitangentOffset) = finalBitangent.Normalized();

	}
	
	delete[] tangents;
	delete[] bitangents;

}