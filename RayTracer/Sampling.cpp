#include "Sampling.h"

#define WRAP_OFFSET 1e-7f

static enum Planes {
	POSX, NEGX,
	POSY, NEGY,
	POSZ, NEGZ
};

inline void SymmetricNormalize(Vec4& v)
{
	v *= 2;
	v -= Vec4(1, 1, 1, 1);
}

inline void AsymmetricNormalize(Vec4& v)
{
	v += Vec4(1, 1, 1, 1);
	v /= 2;
}

inline static Vec4 LinearSample(const Surface& texture, const Vec2& texel)
{

	// enables texture tiling
	float s = texel.s - (int)(texel.s - WRAP_OFFSET);
	float t = texel.t - (int)(texel.t - WRAP_OFFSET);

	// completely regular texture sample
	return texture.GetPixel((int)(s * (texture.GetWidth() - 1)), (int)(t * (texture.GetHeight() - 1)));
}

inline static Vec4 BiLinearSample(const Surface& texture, const Vec2& texel)
{
	// Equations for a Bi Linear Sample
	// From Section 7.5.4, "Mathematics for 3D Game Programming and Computer Graphics", Lengyel

	// enables texture tiling
	float s = texel.s - (int)(texel.s - WRAP_OFFSET);
	float t = texel.t - (int)(texel.t - WRAP_OFFSET);

	// texel location minus half a pixel in x and y
	int i = (texture.GetWidth() * s - 0.5);
	int j = (texture.GetHeight() * t - 0.5);

	// fractional parts of the displaced texel location
	float alpha = i - (int)i;
	float beta = j - (int)j;

	i = (int)i;
	j = (int)j;

	// texture samples of the 4 pixel square
	const Vec4& c1 = texture.GetPixel(i, j);
	const Vec4& c2 = i + 1 >= texture.GetWidth() ? texture.GetPixel(i, j) : texture.GetPixel(i + 1, j);
	const Vec4& c3 = j + 1 >= texture.GetHeight() ? texture.GetPixel(i, j) : texture.GetPixel(i, j + 1);
	const Vec4& c4 = i + 1 >= texture.GetWidth() || j + 1 >= texture.GetHeight() ? texture.GetPixel(i, j) : texture.GetPixel(i + 1, j + 1);

	// weighted average of the 4
	return c1 * (1 - alpha) * (1 - beta) +
		c2 * alpha * (1 - beta) +
		c3 * (1 - alpha) * beta +
		c4 * alpha * beta;
}

Vec3 SampleNormalMap (const Surface& texture, const Vec2& texel)
{
	Vec4 sample = LinearSample(texture, texel);
	SymmetricNormalize(sample);
	return sample.Vec3();
}

Vec4 SampleTexture(const Surface& texture, const Vec2& texel)
{
	return LinearSample(texture, texel);
}

Vec4 SampleCubeMap(const Surface* planes, const Vec3& dir)
{
	// Cube map sampling equations from section 7.5, "Mathematics for 3D Game Programming and Computer Graphics", Lengyel

	float s = dir.s;
	float t = dir.t;
	float p = dir.p;

	const Surface* surfaceToSample = &planes[0];
	float finalS = 0;
	float finalT = 0;

	float absS = fabs(s);
	float absT = fabs(t);
	float absP = fabs(p);

	// figure out the face to sample and the coordinate to sample at
	// determined by sign of coordinate with largest absolute value

	if ( absS >= absT && absS >= absP ) {

		if ( s > 0 ) {
			// positive x
			surfaceToSample = &planes[POSX];
			finalS = 0.5f - p / (2 * s);
			finalT = 0.5f - t / (2 * s);

		}
		else {
			// negative x
			surfaceToSample = &planes[NEGX];
			finalS = 0.5f - p / (2 * s);
			finalT = 0.5f + t / (2 * s);
		}

	}
	else if ( absT >= absS && absT >= absP ) {

		if ( t > 0 ) {
			// positive y
			surfaceToSample = &planes[POSY];
			finalS = 0.5f + s / (2 * t);
			finalT = 0.5f + p / (2 * t);
		}
		else {
			// negative y
			surfaceToSample = &planes[NEGY];
			finalS = 0.5f - s / (2 * t);
			finalT = 0.5f + p / (2 * t);
		}

	}
	else {

		if ( p > 0 ) {
			// positive z
			surfaceToSample = &planes[POSZ];
			finalS = 0.5f + s / (2 * p);
			finalT = 0.5f - t / (2 * p);
		}
		else {
			// negative z
			surfaceToSample = &planes[NEGZ];
			finalS = 0.5f + s / (2 * p);
			finalT = 0.5f + t / (2 * p);
		}
	}

	// don't bother with bilinear sampling, cube maps usually are high resolution
	return LinearSample(*surfaceToSample, { finalS, finalT });

}
