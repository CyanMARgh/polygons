#include "box2.h"

Box2::Box2(float sx, float sy, float x0, float y0) : s(sx, sy), p0(x0, y0) { }
Box2::Box2(vec2 s, vec2 p0) : s(s), p0(p0) { }

vec2 Box2::bl() const { return p0; }
vec2 Box2::tr() const { return p0 + s; }
vec2 Box2::br() const { return {p0.x + s.x, p0.y}; }
vec2 Box2::tl() const { return {p0.x, p0.y + s.y}; }

vec2 Box2::center() const { return p0 + s * .5f; }
vec2 Box2::rad() const { return s * .5f; }

vec2 operator* (const Box2& A, vec2 v) { return A.p0 + A.s * v; }
Box2 operator* (const Box2& A, const Box2& B) { return {A.s * B.s, A * B.p0}; }

Box2 Box2::inv() const { return Box2(1.f / s,  -p0 / s); }