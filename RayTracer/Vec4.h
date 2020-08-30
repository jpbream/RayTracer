#pragma once
#include <iostream>

class Vec3;

class Vec4
{

public:

	union {
		float x;
		float r;
		float s;
	};

	union {
		float y;
		float g;
		float t;
	};

	union {
		float z;
		float b;
		float p;
	};

	union {
		float w;
		float a;
		float q;
	};

	Vec4();
	Vec4(float x, float y, float z, float w);
	Vec4(const Vec4& v);

	Vec4& operator=(const Vec4& v);

	Vec4 operator%(const Vec4& v) const;

	float operator*(const Vec4& v) const;
	Vec4 operator+(const Vec4& v) const;
	Vec4 operator-(const Vec4& v) const;

	Vec4 operator*(float s) const;
	Vec4 operator/(float s) const;
	Vec4& operator*=(float s);
	Vec4& operator/=(float s);

	Vec4& operator+=(const Vec4& v);
	Vec4& operator-=(const Vec4& v);

	Vec4 operator-() const;

	float Length() const;
	Vec4 Normalized() const;

	Vec4 Reflect(const Vec4& normal) const;
	Vec4 Refract(const Vec4& normal, float n1, float n2) const;

	static Vec4 Modulate(const Vec4& v1, const Vec4& v2);
	static Vec4 Lerp(const Vec4& start, const Vec4& end, float alpha);

	Vec3 Vec3() const;
	void Clamp();

};

std::ostream& operator<<(std::ostream& os, const Vec4& v);



