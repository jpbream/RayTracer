#include "Lighting.h"

float FacingFactor(const Vec3& lightDirection, const Vec3& surfaceNormal)
{
	float ff = -lightDirection * surfaceNormal;
	if ( ff < 0 )
		return 0;
	return ff;
}
float SpecularFactor(const Vec3& toLight, const Vec3& surfaceNormal, const Vec3& toCamera, float specularExponent)
{
	// specular reflection model described in
	// "Mathematics for 3D Game Programming and Computer Graphics" by Eric Lengyel
	// Section 7.4

	// vector halfway between to-viewer and to-light vector
	Vec3 halfway = (toLight + toCamera).Normalized();

	// spec factor is how much to scale the specular color by
	float specFactor = surfaceNormal * halfway;
	if ( specFactor < 0 )
		specFactor = 0;
	else
		specFactor = powf(specFactor, specularExponent);

	return surfaceNormal * toLight > 0 ? specFactor : 0;
}
