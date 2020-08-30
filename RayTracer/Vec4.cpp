#include "Vec4.h"
#include "Vec3.h"
#include <math.h>

Vec4::Vec4() : x(0), y(0), z(0), w(1) {}

Vec4::Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

Vec4::Vec4(const Vec4& v) : x(v.x), y(v.y), z(v.z), w(v.w) {}

Vec4& Vec4::operator=(const Vec4& v) {

	x = v.x;
	y = v.y;
	z = v.z;
	w = v.w;

	return *this;

}

Vec4 Vec4::operator%(const Vec4& v) const {

	return { y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x, w};

}

float Vec4::operator*(const Vec4& v) const {

	return x * v.x + y * v.y + z * v.z + w * v.w;
}

Vec4 Vec4::operator+(const Vec4& v) const {

	return { x + v.x, y + v.y, z + v.z, w + v.w };
	
}
Vec4 Vec4::operator-(const Vec4& v) const {

	return { x - v.x, y - v.y, z - v.z, w - v.w };

}

Vec4 Vec4::operator*(float s) const {

	return { x * s, y * s, z * s, w * s };

}
Vec4 Vec4::operator/(float s) const {

	return { x / s, y / s, z / s, w / s };

}
Vec4& Vec4::operator*=(float s) {

	x *= s;
	y *= s;
	z *= s;
	w *= s;

	return *this;

}
Vec4& Vec4::operator/=(float s) {

	x /= s;
	y /= s;
	z /= s;
	w /= s;

	return *this;

}

Vec4& Vec4::operator+=(const Vec4& v) {

	x += v.x;
	y += v.y;
	z += v.z;
	w += v.w;

	return *this;

}
Vec4& Vec4::operator-=(const Vec4& v) {

	x -= v.x;
	y -= v.y;
	z -= v.z;
	w -= v.w;

	return *this;

}

Vec4 Vec4::operator-() const {

	return { -x, -y, -z , -w};

}

float Vec4::Length() const {

	return sqrtf(x * x + y * y + z * z + w * w);

}
Vec4 Vec4::Normalized() const {
	return *this * (1 / Length());
}

Vec4 Vec4::Reflect(const Vec4& normal) const {

	// Equations from "Mathematics for 3D Game Programming and Computer Graphics" by Eric Lengyel

	Vec4 thisNormal = Normalized();
	Vec4 normNormal = normal.Normalized();

	return normNormal * (normNormal * thisNormal * 2) - thisNormal;

}

Vec4 Vec4::Refract(const Vec4& normal, float n1, float n2) const {

	// Equations from "Mathematics for 3D Game Programming and Computer Graphics" by Eric Lengyel

	Vec4 thisNormal = Normalized();
	Vec4 normNormal = normal.Normalized();

	float nRatio = n1 / n2;

	float coefficient = ((normNormal * nRatio) * thisNormal) - (sqrt(1 - (nRatio * nRatio) * (1 - pow(normNormal * thisNormal, 2))));

	return normNormal * coefficient - thisNormal * nRatio;

}

Vec4 Vec4::Modulate(const Vec4& v1, const Vec4& v2) {
	return { v1.r * v2.r, v1.g * v2.g, v1.b * v2.b, v1.a * v2.a };
}

Vec4 Vec4::Lerp(const Vec4& start, const Vec4& end, float alpha)
{
	return start * (1 - alpha) + end * alpha;
}

Vec3 Vec4::Vec3() const {
	return { x, y, z };
}

void Vec4::Clamp()
{
	if ( x < 0 )
		x = 0;
	if ( y < 0 )
		y = 0;
	if ( z < 0 )
		z = 0;
	if ( w < 0 )
		w = 0;

	if ( x > 1 )
		x = 1;
	if ( y > 1 )
		y = 1;
	if ( z > 1 )
		z = 1;
	if ( w > 1 )
		w = 1;
}

std::ostream& operator<<(std::ostream& os, const Vec4& v) {

	os << "[ " << v.x << " " << v.y << " " << v.z << " " << v.w << " ]";
	return os;

}
