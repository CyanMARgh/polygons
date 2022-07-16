#pragma once

#include "primitives.h"
#include "polygon.h"
#include <vector>

typedef	std::vector<vec2s> border;

struct surface {
	std::vector<u32> data;
	vec2u size;
	box2 zone, invzone;

	surface(vec2u size, box2 virtual_zone);

	vec2s proj(vec2 p) const;
	float grid_h(s32 i) const;

	void draw(const border& B, u32 value = ~0);
	void draw(vec2s coord, u32 value = ~0);
	void draw(vec2 p, u32 value = ~0);
	void draw(const point_cloud& cloud, u32 value = ~0);
	void draw(const poly& P, u32 value = ~0);
	//void draw(const poly& P, const monotonic_zones& mz, u32 value = ~0);
	void draw_border(const poly& P, u32 value = ~0);
	void draw(const sliceable_group& P);
	void clear();

	u32& at(vec2u i);
	u32 at(vec2u i) const;
};

border rasterize_monotomics(const surface& S, const poly& P, monotonic_zones::zone z);



