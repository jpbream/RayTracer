#include "Vec2.h"
#include <math.h>

Vec2::Vec2() : x(0), y(0) {}

Vec2::Vec2(float x, float y) : x(x), y(y) {}

Vec2::Vec2(const Vec2& v) : x(v.x), y(v.y) {}

Vec2& Vec2::operator=(const Vec2& v) {

	x = v.x;
	y = v.y;

	return *this;

}

float Vec2::operator*(const Vec2& v) const {

	return x * v.x + y * v.y;
}

Vec2 Vec2::operator+(const Vec2& v) const {

	return { x + v.x, y + v.y };

}
Vec2 Vec2::operator-(const Vec2& v) const {

	return { x - v.x, y - v.y };

}

Vec2 Vec2::operator*(float s) const {

	return { x * s, y * s };

}
Vec2 Vec2::operator/(float s) const {

	return { x / s, y / s };

}
Vec2& Vec2::operator*=(float s) {

	x *= s;
	y *= s;

	return *this;

}
Vec2& Vec2::operator/=(float s) {

	x /= s;
	y /= s;

	return *this;

}

Vec2& Vec2::operator+=(const Vec2& v) {

	x += v.x;
	y += v.y;

	return *this;

}
Vec2& Vec2::operator-=(const Vec2& v) {

	x -= v.x;
	y -= v.y;

	return *this;

}

Vec2 Vec2::operator-() const {

	return { -x, -y };

}

Vec2 Vec2::Lerp(const Vec2& start, const Vec2& end, float alpha)
{
	return start * (1 - alpha) + end * alpha;
}

float Vec2::Length() const {

	return sqrtf(x * x + y * y);

}
Vec2 Vec2::Normalized() const {
	return *this * (1 / Length());
}

Vec2 operator/(float f, const Vec2& v)
{
	return { f / v.x, f / v.y };
}

std::ostream& operator<<(std::ostream& os, const Vec2& v) {

	os << "[ " << v.x << " " << v.y << " ]";
	return os;

}
