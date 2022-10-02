#pragma once

#include "primitives.h"
#include "polygon.h"
#include <vector>

typedef	std::vector<vec2s> Border;

struct Surface {
	std::vector<u32> data;
	vec2u size;
	Box2 zone, invzone;

	Surface(vec2u size, Box2 virtual_zone);

	vec2s proj(vec2 p) const;
	float grid_h(s32 i) const;

	void draw(const Border& B, u32 value = ~0);
	void draw(vec2s coord, u32 value = ~0);
	void draw(vec2 p, u32 value = ~0);
	void draw(const Point_Cloud& cloud, u32 value = ~0);

	void rasterize_borders(const Poly& P, std::vector<Border>& borders, u32& S);
	void draw(std::vector<Border>& border, u32 value, u32 S);
	void draw(const Poly& P, u32 value = ~0);
	void draw(const Holey_Poly& P, u32 value = ~0);

	void draw_border(const Poly& P, u32 value = ~0);
	void draw(const Sliceable_Group& P);
	void clear();

	u32& at(vec2u i);
	u32 at(vec2u i) const;
};

Border rasterize_monotomics(const Surface& S, const Poly& P, Monotonic_Zones::zone z);



