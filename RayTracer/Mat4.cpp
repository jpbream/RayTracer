#include "Mat4.h"
#include "Mat3.h"
#include "Utility.h"
#include <memory>
#include <math.h>
#include <immintrin.h>

#define PI 3.14159265358979323846

const Mat4 Mat4::Identity{ 
	{ 1, 0, 0, 0 }, 
	{ 0, 1, 0, 0 }, 
	{ 0, 0, 1, 0 }, 
	{0, 0, 0, 1} 
};

const Mat4 Mat4::Viewport{
	{.5, 0, 0, 0},
	{0, -.5, 0, 0},
	{0, 0, .5, 0},
	{.5, .5, .5, 1}
};

Mat4::Mat4() {
	cols[0] = { 1, 0, 0, 0 };
	cols[1] = { 0, 1, 0, 0 };
	cols[2] = { 0, 0, 1, 0 };
	cols[3] = { 0, 0, 0, 1 };
}

Mat4::Mat4(const Vec4& c1, const Vec4& c2, const Vec4& c3, const Vec4& c4)
{

	cols[0] = c1;
	cols[1] = c2;
	cols[2] = c3;
	cols[3] = c4;

}

Mat4::Mat4(const Mat4& m) {

	memcpy(data, m.data, sizeof(Mat4));

}

Mat4& Mat4::operator=(const Mat4& m) {

	memcpy(data, m.data, sizeof(Mat4));
	return *this;

}

float& Mat4::operator()(int r, int c) {

	// data is stored column major
	return data[c * 4 + r];

}
float Mat4::operator()(int r, int c) const {

	// data is stored column major
	return data[c * 4 + r];

}

Vec4& Mat4::operator[](int c) {

	return cols[c];

}
Vec4 Mat4::operator[](int c) const {

	return cols[c];

}

Mat4 Mat4::operator*(const Mat4& m) const {

	Mat4 productMat;
	float* ptr = (float*)&productMat;

	__m256 row12 = _mm256_set_ps(data[12], data[8], data[4], data[0], data[13], data[9], data[5], data[1]);
	__m256 row34 = _mm256_set_ps(data[14], data[10], data[6], data[2], data[15], data[11], data[7], data[3]);

	// walk down columns
	for ( int i = 0; i < 4; ++i ) {

		// can get two results per multiply operation if using 256 bit vectors
		__m256 col = _mm256_set_ps(m.cols[i].w, m.cols[i].z, m.cols[i].y, m.cols[i].x, 
								   m.cols[i].w, m.cols[i].z, m.cols[i].y, m.cols[i].x);

		// calculate entire resultant column
		__m256 prod1 = _mm256_mul_ps(col, row12);
		__m256 prod2 = _mm256_mul_ps(col, row34);

		// first value is the high 4 values
		*(ptr++) = HAdd4(_mm256_extractf128_ps(prod1, 1));

		// second value is the low 4 values
		*(ptr++) = HAdd4(_mm256_castps256_ps128(prod1));
		
		*(ptr++) = HAdd4(_mm256_extractf128_ps(prod2, 1));
		*(ptr++) = HAdd4(_mm256_castps256_ps128(prod2));
		
	}
	
	return productMat;
}

Vec4 Mat4::operator*(const Vec4& v) const {

	Vec4 thisRow1(data[0], data[4], data[8], data[12]);
	Vec4 thisRow2(data[1], data[5], data[9], data[13]);
	Vec4 thisRow3(data[2], data[6], data[10], data[14]);
	Vec4 thisRow4(data[3], data[7], data[11], data[15]);

	return { thisRow1 * v, thisRow2 * v, thisRow3 * v, thisRow4 * v };

}

Mat4 Mat4::operator*(float c) const {

	return { cols[0] * c, cols[1] * c, cols[2] * c, cols[3] * c };

}

float Mat4::GetDeterminant() const {

	// walk across the fourth row, use the 3x3 determinant method for
	// each submatrix

	Vec4 cp0 = cols[2] % cols[3];
	float term0 = -cols[0].w * (cols[1].x * cp0.x + cols[1].y * cp0.y + cols[1].z * cp0.z);

	// both the first and second columns make use of the cross product of the last 2 columns
	Vec4& cp1 = cp0;
	float term1 = cols[1].w * (cols[0].x * cp1.x + cols[0].y * cp1.y + cols[0].z * cp1.z);

	Vec4 cp2 = cols[1] % cols[3];
	float term2 = -cols[2].w * (cols[0].x * cp2.x + cols[0].y * cp2.y + cols[0].z * cp2.z);

	Vec4 cp3 = cols[1] % cols[2];
	float term3 = cols[3].w * (cols[0].x * cp3.x + cols[0].y * cp3.y + cols[0].z * cp3.z);

	return (term0 + term1) + (term2 + term3);

}

typedef union { __m128 vec; float arr[4]; } row;

static inline void InverseHelper(row& pivot, row& target, row& inversePivot, row& inverseTarget, int column) {

	static __m128 factorBuf, multBuf;

	// how much to multiply the pivot by, to add to the target to zero it out
	float factor = target.arr[column] / pivot.arr[column];
	factorBuf = _mm_set1_ps(-factor);

	// multiply the pivot row by the factor
	multBuf = _mm_mul_ps(pivot.vec, factorBuf);

	// add this multiplied row to the target, zeroing out the column
	target.vec = _mm_add_ps(target.vec, multBuf);

	// do the same on the identity to inverse vectors
	multBuf = _mm_mul_ps(inversePivot.vec, factorBuf);
	inverseTarget.vec = _mm_add_ps(inverseTarget.vec, multBuf);

	// divide row by leading coefficient
	factor = pivot.arr[column];
	factorBuf = _mm_set1_ps(factor);

	pivot.vec = _mm_div_ps(pivot.vec, factorBuf);

	// do the same for the inverse row
	inversePivot.vec = _mm_div_ps(inversePivot.vec, factorBuf);

}

static inline void SwapRow(row& row1, row& row2) {

	row temp = row1;
	row1 = row2;
	row2 = temp;

}

Mat4 Mat4::GetInverse() const {

	row row0, inverse0;
	row row1, inverse1;
	row row2, inverse2;
	row row3, inverse3;

	// data is stored in reverse in the 128 vectors for some reason
	row0.vec = _mm_set_ps(data[12], data[8], data[4], data[0]);
	row1.vec = _mm_set_ps(data[13], data[9], data[5], data[1]);
	row2.vec = _mm_set_ps(data[14], data[10], data[6], data[2]);
	row3.vec = _mm_set_ps(data[15], data[11], data[7], data[3]);

	// data is stored in reverse in the 128 vectors for some reason
	inverse0.vec = _mm_set_ps(0, 0, 0, 1);
	inverse1.vec = _mm_set_ps(0, 0, 1, 0);
	inverse2.vec = _mm_set_ps(0, 1, 0, 0);
	inverse3.vec = _mm_set_ps(1, 0, 0, 0);

	// order rows
	if (fabs(row1.arr[0]) > fabs(row0.arr[0])) {
		SwapRow(row1, row0);
		SwapRow(inverse1, inverse0);
	}
	if (fabs(row2.arr[0]) > fabs(row0.arr[0])) {
		SwapRow(row2, row0);
		SwapRow(inverse2, inverse0);
	}
	if (fabs(row3.arr[0]) > fabs(row0.arr[0])) {
		SwapRow(row3, row0);
		SwapRow(inverse3, inverse0);
	}

	// zero out column 0
	InverseHelper(row0, row1, inverse0, inverse1, 0);
	InverseHelper(row0, row2, inverse0, inverse2, 0);
	InverseHelper(row0, row3, inverse0, inverse3, 0);

	// zero out column 1
	InverseHelper(row1, row0, inverse1, inverse0, 1);
	InverseHelper(row1, row2, inverse1, inverse2, 1);
	InverseHelper(row1, row3, inverse1, inverse3, 1);

	// zero out column 2
	InverseHelper(row2, row0, inverse2, inverse0, 2);
	InverseHelper(row2, row1, inverse2, inverse1, 2);
	InverseHelper(row2, row3, inverse2, inverse3, 2);

	// zero out column 3
	InverseHelper(row3, row0, inverse3, inverse0, 3);
	InverseHelper(row3, row1, inverse3, inverse1, 3);
	InverseHelper(row3, row2, inverse3, inverse2, 3);

	return { 
		{inverse0.arr[0], inverse1.arr[0], inverse2.arr[0], inverse3.arr[0]},
	    {inverse0.arr[1], inverse1.arr[1], inverse2.arr[1], inverse3.arr[1]},
	    {inverse0.arr[2], inverse1.arr[2], inverse2.arr[2], inverse3.arr[2]},
	    {inverse0.arr[3], inverse1.arr[3], inverse2.arr[3], inverse3.arr[3]}
	};

}

Mat4 Mat4::GetTranspose() const {
	return
	{
		{data[0], data[4], data[8], data[12]},
		{data[1], data[5], data[9], data[13]},
		{data[2], data[6], data[10], data[14]},
	    {data[3], data[7], data[11], data[15]}
	};
}

Mat3 Mat4::Truncate() const {

	return { {cols[0].x, cols[0].y, cols[0].z},
			 {cols[1].x, cols[1].y, cols[1].z},
			 {cols[2].x, cols[2].y, cols[2].z}
	};

}

Mat4 Mat4::Get3DTranslation(float dx, float dy, float dz) {

	return { {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {dx, dy, dz, 1} };

}
Mat4 Mat4::GetRotation(float rx, float ry, float rz) {

	__m128 data = _mm_set_ps(0, rz, ry, rx);

	__m128 sins = _mm_sin_ps(data);
	__m128 coss = _mm_cos_ps(data);

	Mat4 matX({ 1, 0, 0, 0 }, { 0, coss.m128_f32[0], sins.m128_f32[0], 0 }, { 0, -sins.m128_f32[0], coss.m128_f32[0], 0 }, {0, 0, 0, 1});
	Mat4 matY({ coss.m128_f32[1], 0, -sins.m128_f32[1], 0 }, { 0, 1, 0, 0 }, { sins.m128_f32[1], 0, coss.m128_f32[1], 0 }, {0, 0, 0, 1});
	Mat4 matZ({ coss.m128_f32[2], sins.m128_f32[2], 0, 0 }, { -sins.m128_f32[2], coss.m128_f32[2], 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 });

	return matZ * matY * matX;
}
Mat4 Mat4::GetScale(float sx, float sy, float sz) {

	return { {sx, 0, 0, 0}, {0, sy, 0, 0}, {0, 0, sz, 0}, {0, 0, 0, 1} };

}

Mat4 Mat4::GetPerspectiveProjection(float n, float f, float l, float r, float t, float b)
{
	return { {2 * n / (r - l), 0, 0, 0}, 
	         {0, 2 * n / (t - b), 0, 0}, 
	         {(r + l) / (r - l), (t + b) / (t - b), -(f + n) / (f - n), -1}, 
	         {0, 0, -2 * n * f / (f - n), 0} 
	};
}

Mat4 Mat4::GetPerspectiveProjection(float n, float f, float aspect, float fovDegrees, Frustum& outputFrustum)
{
	float fov = fovDegrees * PI / 180.0f;
	float e = 1.0f / tanf(fov / 2.0f);

	float l = -n / e;
	float r = n / e;
	float t = aspect * n / e;
	float b = -aspect * n / e;

	outputFrustum.near = n;
	outputFrustum.far = f;
	outputFrustum.left = l;
	outputFrustum.right = r;
	outputFrustum.top = t;
	outputFrustum.bottom = b;

	return GetPerspectiveProjection(n, f, l, r, t, b);

}

Mat4 Mat4::GetOrthographicProjection(float n, float f, float l, float r, float t, float b)
{
	
	return {
		{2 / (r - l), 0, 0, 0},
		{0, 2 / (t - b), 0, 0},
		{0, 0, -2 / (f - n), 0},
		{-(r + l) / (r - l), -(t + b) / (t  - b), -(f + n) / (f - n), 1}
	};

}

std::ostream& operator<<(std::ostream& os, const Mat4& m) {

	os << "[ " << m.data[0] << " " << m.data[4] << " " << m.data[8] << " " << m.data[12] << " ]" << std::endl;
	os << "| " << m.data[1] << " " << m.data[5] << " " << m.data[9] << " " << m.data[13] << " |" << std::endl;
	os << "| " << m.data[2] << " " << m.data[6] << " " << m.data[10] << " " << m.data[14] << " |" << std::endl;
	os << "[ " << m.data[3] << " " << m.data[7] << " " << m.data[11] << " " << m.data[15] << " ]";

	return os;

}
