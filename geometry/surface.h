#pragma once

#include "primitives.h"
#include "polygon.h"
#include <vector>

typedef	std::vector<vec2s> border;

struct Surface {
	std::vector<u32> data;
	vec2u size;
	Box2 zone, invzone;

	Surface(vec2u size, Box2 virtual_zone);

	vec2s proj(vec2 p) const;
	float grid_h(s32 i) const;

	void draw(const border& B, u32 value = ~0);
	void draw(vec2s coord, u32 value = ~0);
	void draw(vec2 p, u32 value = ~0);
	void draw(const Point_Cloud& cloud, u32 value = ~0);
	void draw(const Poly& P, u32 value = ~0);
	//void draw(const Poly& P, const Monotonic_Zones& mz, u32 value = ~0);
	void draw_border(const Poly& P, u32 value = ~0);
	void draw(const Sliceable_Group& P);
	void clear();

	u32& at(vec2u i);
	u32 at(vec2u i) const;
};

border rasterize_monotomics(const Surface& S, const Poly& P, Monotonic_Zones::zone z);



