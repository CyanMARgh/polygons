#pragma once
#include "geometry.h"

struct line {
	vec2 a, b;
};
struct intersection {
	float t1, t2;
	s32 id;
};
struct circle {
	vec2 f1 = {};
	vec2 f2 = {};

	bool inside(vec2 p) const;
	float rad() const;
	vec2 center() const;
};
struct box2 {
	vec2 s = {1.f, 1.f}, p0 = {0.f, 0.f}; 

	box2(float sx = 1.f, float sy = 1.f, float x0 = 0.f, float y0 = 0.f);
	box2(vec2 s, vec2 p0);

	vec2 bl() const;
	vec2 tr() const;
	vec2 br() const;
	vec2 tl() const;

	vec2 center() const;
	vec2 rad() const;
	box2 inv() const;

	vec2 operator* (vec2 v) const;
	box2 operator* (const box2& B) const;
};

struct mat2x2 {
	float a, b, c, d;

	static mat2x2 from_rows(vec2 ab, vec2 cd);
	static mat2x2 from_columns(vec2 ac, vec2 bd);

	mat2x2 inv() const;	
	float det() const;
	mat2x2 transposed() const;

	mat2x2 operator*(const mat2x2& B) const;
	vec2 operator*(vec2 v) const;
};