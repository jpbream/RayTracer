#pragma once
#include "Vec3.h"

float FacingFactor(const Vec3& lightDirection, const Vec3& surfaceNormal);

float SpecularFactor(const Vec3& toLight, const Vec3& surfaceNormal, const Vec3& toCamera, float specularExponent);

