#pragma once
#include "Vec2.h"
#include "Vec3.h"
#include "Vec4.h"
#include "Surface.h"

Vec4 LinearSample(const Surface& texture, const Vec2& texel);

Vec4 BiLinearSample(const Surface& texture, const Vec2& texel);

Vec4 SampleCubeMap(const Surface* planes, const Vec3& dir);
