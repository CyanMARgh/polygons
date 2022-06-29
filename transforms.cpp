#include "transforms.h"

box2::box2(float sx, float sy, float x0, float y0) : s(sx, sy), p0(x0, y0) { }
box2::box2(vec2 s, vec2 p0) : s(s), p0(p0) { }

vec2 box2::bl() const { return p0; }
vec2 box2::tr() const { return p0 + s; }
vec2 box2::br() const { return {p0.x + s.x, p0.y}; }
vec2 box2::tl() const { return {p0.x, p0.y + s.y}; }

vec2 box2::center() const { return p0 + s * .5f; }
vec2 box2::rad() const { return s * .5f; }

vec2 operator* (const box2& A, vec2 v) { return A.p0 + A.s * v; }
box2 operator* (const box2& A, const box2& B) { return {A.s * B.s, A * B.p0}; }

box2 box2::inv() const { return box2(1.f / s,  -p0 / s); }

mat2x2 mat2x2::from_rows(vec2 ab, vec2 cd) {
	return {ab.x, ab.y, cd.x, cd.y};
}
mat2x2 mat2x2::from_columns(vec2 ac, vec2 bd) {
	return {ac.x, bd.x, ac.x, bd.y};
}

mat2x2 mat2x2::inv() const {
	float D = det();
	return {d / D, -b / D, -c / D, a / D};
}
float mat2x2::det() const {
	return a * d - b * c;
}
mat2x2 mat2x2::transposed() const {
	return {a, c, b, d};
}
mat2x2 operator*(const mat2x2& A, const mat2x2& B) {
	return { A.a * B.a + A.b * B.c, A.a * B.b + A.b * B.d,
			 A.c * B.a + A.d * B.c, A.c * B.b + A.d * B.d};
}
vec2 operator*(const mat2x2& A, vec2 v) {
	return { A.a * v.x + A.b * v.y, A.c * v.x + A.d * v.y };
}
