#pragma once

#include "geometry.h"
#include "utils.h"

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
//	void draw(sf::RenderWindow& rwin, sf::CircleShape& spr, box2 box) const;

	float rad() const;
	vec2 center() const;
};
struct cloud_range {
	point_cloud* source;
	u32 a, b;

	s32 size() const; 
	vec2 at(u32 i) const;
};