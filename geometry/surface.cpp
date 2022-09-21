#include "surface.h"
#include "utils.h"
#include "sliceable_group.h"

border rasterize_monotomics(const Surface& S, const Poly& P, Monotonic_Zones::zone z) {
	vec2s pi_a = S.proj(P[z.a]), pi_b = S.proj(P[z.b]);
	if (pi_a.y == pi_b.y) return {pi_a};
	border B = {pi_a, pi_b};

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

void Surface::draw(const border& B, u32 value) {
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
		border B = rasterize_monotomics(*this, P, z);
		draw(B, value);
	}
}
void Surface::draw(const Poly& P, u32 value) {
	if(P.size < 3) return;
	auto mz = geom::divide_to_monotonics(P);
	std::vector<border> borders = {};
	u32 S = 0;
	for(auto z : mz.parts) {
		border B = rasterize_monotomics(*this, P, z);
		S += B.size();
		borders.push_back(std::move(B));
	}
	std::vector<vec2s> BB(S);
	for(u32 i = 0, s = 0, n = borders.size(); i < n; i++) {
		std::copy(borders[i].begin(), borders[i].end(), BB.begin() + s);
		s += borders[i].size();
	}
	if(S % 2)  {
		printf("odd number of border pixels\n");
		s32 h_sum = 0;
		for(u32 i = 0, n = borders.size(); i < n; i++) {
			vec2s pi_a = proj(P[mz.parts[i].a]), pi_b = proj(P[mz.parts[i].b]);
			printf("%d-%d\n", pi_a.y, pi_b.y);
			h_sum += pi_b.y - pi_a.y;
		}
		printf("h_sum: %d\n", h_sum);
		return;
		//throw std::runtime_error("odd number of border pixels\n");
	}
	sort(BB.begin(), BB.end(), [] (vec2s a, vec2s b) { return a.y == b.y ? a.x < b.x : a.y < b.y; });
	for(u32 i = 0; i < S-1; i += 2) {
		vec2s p0 = BB[i], p1 = BB[i + 1];
		if(p0.y != p0.y) {
			throw std::runtime_error("border pixels heigth missmatch\n");
		} else if(p0.y < 0) {
			continue;
		} else if(p0.y >= size.y) {
			break;
		}
		if(p1.x < 0) continue;
		if(p0.x < 0) p0.x = 0;
		if(p1.x >= size.x) p1.x = size.x;
		for(s32 x = p0.x; x <= p1.x; x++) draw(vec2s(x, p0.y), value);
	}
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










