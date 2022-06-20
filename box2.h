#pragma once
#include "utils.h"

struct Box2 {
	vec2 s = {1.f, 1.f}, p0 = {0.f, 0.f}; 

	Box2(float sx = 1.f, float sy = 1.f, float x0 = 0.f, float y0 = 0.f);
	Box2(vec2 s, vec2 p0);

	vec2 bl() const;
	vec2 tr() const;
	vec2 br() const;
	vec2 tl() const;

	vec2 center() const;
	vec2 rad() const;

	Box2 inv() const;
};

vec2 operator* (const Box2& A, vec2 v);
Box2 operator* (const Box2& A, const Box2& B);
