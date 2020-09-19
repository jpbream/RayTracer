#pragma once
#include "Vec2.h"
#include "Vec3.h"
#include "Vec4.h"
#include "Surface.h"

Vec3 SampleNormalMap (const Surface& texture, const Vec2& texel);

Vec4 SampleTexture(const Surface& texture, const Vec2& texel);

Vec4 SampleCubeMap(const Surface* planes, const Vec3& dir);

