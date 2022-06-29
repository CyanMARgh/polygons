#pragma once
#include "utils.h"

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
};

vec2 operator* (const box2& A, vec2 v);
box2 operator* (const box2& A, const box2& B);

struct mat2x2 {
	float a, b, c, d;

	static mat2x2 from_rows(vec2 ab, vec2 cd);
	static mat2x2 from_columns(vec2 ac, vec2 bd);

	mat2x2 inv() const;	
	float det() const;
	mat2x2 transposed() const;
};

mat2x2 operator*(const mat2x2& A, const mat2x2& B);
vec2 operator*(const mat2x2& A, vec2 v);