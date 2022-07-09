#include "primitives.h"
#include "utils.h"

box2::box2(float sx, float sy, float x0, float y0) : s(sx, sy), p0(x0, y0) { }
box2::box2(vec2 s, vec2 p0) : s(s), p0(p0) { }

vec2 box2::bl() const { return p0; }
vec2 box2::tr() const { return p0 + s; }
vec2 box2::br() const { return {p0.x + s.x, p0.y}; }
vec2 box2::tl() const { return {p0.x, p0.y + s.y}; }

vec2 box2::center() const { return p0 + s * .5f; }
vec2 box2::rad() const { return s * .5f; }

vec2 box2::operator* (vec2 v) const { return p0 + s * v; }
box2 box2::operator* (const box2& B) const { return {s * B.s, *this * B.p0}; }

box2 box2::inv() const { return box2(1.f / s,  -p0 / s); }

mat2x2 mat2x2::from_rows(vec2 ab, vec2 cd) { return {ab.x, ab.y, cd.x, cd.y}; }
mat2x2 mat2x2::from_columns(vec2 ac, vec2 bd) { return {ac.x, bd.x, ac.x, bd.y}; }

mat2x2 mat2x2::inv() const { float D = det(); return {d / D, -b / D, -c / D, a / D}; }
float mat2x2::det() const { return a * d - b * c; }
mat2x2 mat2x2::transposed() const { return {a, c, b, d}; }
mat2x2 mat2x2::operator*(const mat2x2& B) const { return { a * B.a + b * B.c, a * B.b + b * B.d, c * B.a + d * B.c, c * B.b + d * B.d}; }
vec2 mat2x2::operator*(vec2 v) const { return { a * v.x + b * v.y, c * v.x + d * v.y }; }

bool circle::inside(vec2 p) const { return len2(p - f1) + len2(p - f2) <= len2(f2 - f1); }
float circle::rad() const { return len(f2 - f1) * .5f; }
vec2 circle::center() const { return (f1 + f2) * .5f; }