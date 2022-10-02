#pragma once
#include "geometry.h"

struct Line {
	vec2 a, b;
};
struct Intersection {
	float t1, t2;
	s32 id, pid = 0;
};
struct Circle {
	vec2 f1 = {};
	vec2 f2 = {};

	static Circle make(vec2 center, float R);
	bool inside(vec2 p) const;
	float rad() const;
	vec2 center() const;
};
struct Box2 {
	vec2 s = {1.f, 1.f}, p0 = {0.f, 0.f}; 

	Box2(float sx = 1.f, float sy = 1.f, float x0 = 0.f, float y0 = 0.f);
	Box2(vec2 s, vec2 p0 = {0.f, 0.f});

	vec2 bl() const;
	vec2 tr() const;
	vec2 br() const;
	vec2 tl() const;

	vec2 center() const;
	vec2 rad() const;
	Box2 inv() const;

	vec2 operator* (vec2 v) const;
	Box2 operator* (const Box2& B) const;
};

struct Mat2x2 {
	float a, b, c, d;

	static Mat2x2 from_rows(vec2 ab, vec2 cd);
	static Mat2x2 from_columns(vec2 ac, vec2 bd);

	Mat2x2 inv() const;	
	float det() const;
	Mat2x2 transposed() const;

	Mat2x2 operator*(const Mat2x2& B) const;
	vec2 operator*(vec2 v) const;
};