#include "surface.h"
#include "utils.h"
#include "sliceable_group.h"
#include "holey_polygon.h"

Border rasterize_monotomics(const Surface& S, const Poly& P, Monotonic_Zones::zone z) {
	vec2s pi_a = S.proj(P[z.a]), pi_b = S.proj(P[z.b]);
	if (pi_a.y == pi_b.y) return {pi_a};
	Border B = {pi_a, pi_b};

	auto find_x = [&S](vec2 a, vec2 b, float y) {
		return  S.proj(vec2(lerp(a.x, b.x, rlerp(a.y, b.y, y)), y)).x;
	};

	if(z.type == Seg_Type::UP) {
		for(s32 i = pi_a.y + 1, j = z.a; i < pi_b.y; i++) {
			float hj, hi = S.grid_h(i);
			while(j < z.b && (hj = P[j + 1].y) < hi) j++;
			B.push_back({find_x(P[j], P[j + 1], hi), i});
		}
	} else {
		for(s32 i = pi_a.y - 1, j = z.a; i > pi_b.y; i--) {
			float hj, hi = S.grid_h(i);
			while(j < z.b && (hj = P[j + 1].y) > hi) j++;
			B.push_back({find_x(P[j], P[j + 1], hi), i});
		}
	}
	return B;
}

vec2s Surface::proj(vec2 p) const {
	p = zone * p;
	return {(s32)p.x, (s32)p.y};
}
float Surface::grid_h(s32 i) const { return (invzone * vec2(0, i)).y; }
u32& Surface::at(vec2u i) { return data[i.x + size.x * i.y]; }
u32 Surface::at(vec2u i) const { return data[i.x + size.x * i.y]; }
void Surface::draw(vec2s coord, u32 value) {
	if(coord.x >= 0 && coord.y >= 0 && coord.x < size.x && coord.y < size.y) {
		at((vec2u)coord) = value;
	}
}
void Surface::draw(vec2 p, u32 value) {
	draw(proj(p), value);
}
void Surface::draw(const Point_Cloud& cloud, u32 value) {
	for(auto p : cloud) { 
		draw(p, value);
	}
}

void Surface::draw(const Border& B, u32 value) {
	for(auto pi : B) draw(pi, value);
}

Surface::Surface(vec2u size, Box2 virtual_zone) : size(size) {
	zone = Box2((vec2)size) * virtual_zone.inv();
	invzone = zone.inv();
	data = std::vector<u32>(size.x * size.y, 0);
}

void Surface::draw_border(const Poly& P, u32 value) {
	auto mz = geom::divide_to_monotonics(P);
	for(auto z : mz.parts) {
		Border B = rasterize_monotomics(*this, P, z);
		draw(B, value);
	}
}
void Surface::rasterize_borders(const Poly& P, std::vector<Border>& borders, u32& S) {
	auto mz = geom::divide_to_monotonics(P);
	u32 dS = 0;
	std::vector<Border> extra_borders;
	for(auto z : mz.parts) {
		Border B = rasterize_monotomics(*this, P, z);
		dS += B.size();
		extra_borders.push_back(std::move(B));
	}
	//TODO fix it
	if(dS % 2 == 0) {
		S += dS;
		for(auto b : extra_borders) {
			borders.push_back(b);
		}
	}
}
void Surface::draw(std::vector<Border>& borders, u32 value, u32 S) {
	// printf("(0)\n");
	std::vector<vec2s> BB(S);
	// printf("(3)\n");
	for(u32 i = 0, s = 0, n = borders.size(); i < n; i++) {
		// printf("(4): %d %d %d\n", i, s, n);
		std::copy(borders[i].begin(), borders[i].end(), BB.begin() + s);
		s += borders[i].size();
		// printf("(5)\n");
	}
	// printf("(2)\n");
	if(S % 2)  {
		return;
		throw std::runtime_error("odd number of Border pixels\n");
	}
	// printf("(9)\n");
	sort(BB.begin(), BB.end(), [] (vec2s a, vec2s b) { return a.y == b.y ? a.x < b.x : a.y < b.y; });
	// printf("(13), S = %d\n", S);
	for(u32 i = 0; i + 1 < S; i += 2) {
		vec2s p0 = BB[i], p1 = BB[i + 1];
		// printf("(10)\n");
		if(p0.y != p0.y) {
			throw std::runtime_error("Border pixels heigth missmatch\n");
		} else if(p0.y < 0) {
			continue;
		} else if(p0.y >= size.y) {
			break;
		}
		// printf("(11)\n");
		if(p1.x < 0) continue;
		if(p0.x < 0) p0.x = 0;
		if(p1.x >= size.x) p1.x = size.x;
		for(s32 x = p0.x; x <= p1.x; x++) draw(vec2s(x, p0.y), value);
		// printf("(12)\n");
	}	
	// printf("(1)\n");
}
void Surface::draw(const Poly& P, u32 value) {
	if(P.size < 3) return;
	std::vector<Border> borders = {};
	u32 S = 0;
	rasterize_borders(P, borders, S);
	draw(borders, value, S);
}
void Surface::draw(const Holey_Poly& HP, u32 value) {
	if(HP.outer.size < 3) return;
	std::vector<Border> borders = {};
	u32 S = 0;
	// printf("(5)\n");
	rasterize_borders(HP.outer, borders, S);
	// printf("(6)\n");
	for(auto& Pi : HP.holes) {
		if(Pi.size < 3) continue;
		rasterize_borders(Pi, borders, S);
	}
	// printf("(7)\n");
	draw(borders, value, S);
	// printf("(8)\n");
}

void Surface::draw(const Sliceable_Group& SG) {
	u32 n = SG.polys.size();
	bool m = SG.mode == Sliceable_Group::MOVE && SG.selected;
	for(u32 i = 0; i + m < n; i++) {
		//printf("DRAW: %d ", i);
		draw(SG.polys[SG.order[i]], SG.colors[SG.order[i]]);
	} 
	if(m) {
		vec2 delta = SG.mouse_curr - SG.mouse_buf;
		Poly T_m = *(SG.top());
		for(u32 i = 0; i < T_m.size; i++) {
			T_m[i] += delta;
		}
		draw(T_m, SG.colors[SG.order[n - 1]]);
	}
	// draw with delta
}


void Surface::clear() { std::fill(data.begin(), data.end(), 0); }










