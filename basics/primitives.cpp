#include "primitives.h"
#include "utils.h"

Box2::Box2(float sx, float sy, float x0, float y0) : s(sx, sy), p0(x0, y0) { }
Box2::Box2(vec2 s, vec2 p0) : s(s), p0(p0) { }

vec2 Box2::bl() const { return p0; }
vec2 Box2::tr() const { return p0 + s; }
vec2 Box2::br() const { return {p0.x + s.x, p0.y}; }
vec2 Box2::tl() const { return {p0.x, p0.y + s.y}; }

vec2 Box2::center() const { return p0 + s * .5f; }
vec2 Box2::rad() const { return s * .5f; }

vec2 Box2::operator* (vec2 v) const { return p0 + s * v; }
Box2 Box2::operator* (const Box2& B) const { return {s * B.s, *this * B.p0}; }

Box2 Box2::inv() const { return Box2(1.f / s,  -p0 / s); }

Mat2x2 Mat2x2::from_rows(vec2 ab, vec2 cd) { return {ab.x, ab.y, cd.x, cd.y}; }
Mat2x2 Mat2x2::from_columns(vec2 ac, vec2 bd) { return {ac.x, bd.x, ac.x, bd.y}; }

Mat2x2 Mat2x2::inv() const { float D = det(); return {d / D, -b / D, -c / D, a / D}; }
float Mat2x2::det() const { return a * d - b * c; }
Mat2x2 Mat2x2::transposed() const { return {a, c, b, d}; }
Mat2x2 Mat2x2::operator*(const Mat2x2& B) const { return { a * B.a + b * B.c, a * B.b + b * B.d, c * B.a + d * B.c, c * B.b + d * B.d}; }
vec2 Mat2x2::operator*(vec2 v) const { return { a * v.x + b * v.y, c * v.x + d * v.y }; }

Circle Circle::make(vec2 center, float R) {
	return {center + vec2(R, 0), center - vec2(R, 0)};
}
bool Circle::inside(vec2 p) const { return len2(p - f1) + len2(p - f2) <= len2(f2 - f1); }
float Circle::rad() const { return len(f2 - f1) * .5f; }
vec2 Circle::center() const { return (f1 + f2) * .5f; }