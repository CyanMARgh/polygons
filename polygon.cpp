#include "polygon.h"
#include <algorithm>

//base
poly::poly() {
	points = {}, size = 0;
}
poly::poly(std::vector<vec2> other) : points(std::move(other)) { 
	size = points.size();
}
poly& poly::operator=(std::vector<vec2> other) {
	points = std::move(other);
	size = other.size();
	return *this;
}
poly::poly(const poly& P) {
	points = P.points;
	size = P.size;
}
void poly::add(vec2 p) {
	points.push_back(p);
	++size;
}
vec2& poly::operator[](s32 i) {
	return points[mmod(i, points.size())];
}
vec2 poly::operator[](s32 i) const {
	return points[mmod(i, points.size())];
}

// is inside
intersection monotonic_zones::inspect_zone(zone z, const poly& P, vec2 p, vec2 n) {
	float py = dot(p, n), ay = dot(P[z.a], n), by = dot(P[z.b], n), cy;
	float mi = fmin(ay, by), ma = fmax(ay, by);

	if(py <= mi || py >= ma || z.type == seg_type::ANY) return {0.f, 0.f, -1};
	bool u_d = z.type == seg_type::UP;

	for(u32 c;;) {
		c = (z.a + z.b) / 2;
		cy = dot(P[c], n);
		if(z.a == c) break;
		((dot(p, n) < cy) ^ u_d ? z.a : z.b) = c;
	}

	vec2 A = P[z.a], B = P[z.b];
	float t1 = rlerp(dot(A, n), dot(B, n), dot(p, n));
	float t2 = lerp(cross(A, n), cross(B, n), t1) - cross(p, n);

	return {t1, t2, z.a};	
}
monotonic_zones poly::divide_to_monotonics(vec2 n) const {
	if(!size) return {};

	std::vector<monotonic_zones::zone> parts;
	s32 i0 = 0;

	seg_type type0, type_temp;
	while((type0 = get_seg_type(i0, n)) == seg_type::ANY && i0 < size) i0++;
	if(i0 == size) return {{{0, (s32)size, type0}}};
	i0++;
	while(type_temp = get_seg_type(i0, n), (type_temp == seg_type::ANY || type_temp == type0) && i0 <= size) i0++;

	for(s32 i = i0, is = i0; i < i0 + size; i++) {
		seg_type type_next = get_seg_type(i + 1, n);
		if(type_next == seg_type::ANY) continue;
		if(type_temp != type_next) {
			parts.push_back({is, i + 1, type_temp});
			type_temp = type_next, is = i + 1;
		}
	}
	return {parts};
}
s32 poly::is_inside_val(const monotonic_zones& mz, vec2 p, vec2 n) const {
	s32 s = 0;
	for(auto z : mz.parts) {
		auto r = monotonic_zones::inspect_zone(z, *this, p, n);
		if(r.id != -1) s += (r.t2 < 0) * (z.type == seg_type::UP ? 1 : -1);
	}
	return s;
}
seg_type poly::get_seg_type(s32 i, vec2 n) const {
	float q = dot(n, (*this)[i+1]-(*this)[i]);
	return q > 0 ? seg_type::UP : q < 0 ? seg_type::DOWN : seg_type::ANY; 
}
void monotonic_zones::print() {
	for(auto z : parts) {
		printf("[%d,%d] ", z.a, z.b);
	}
	printf("\n");
}

//slice
intersection_list poly::find_intersections(line l) const {
	if(l.b == l.a) return {};
	vec2 ab = l.b - l.a;
	vec2 n = rrot(ab);
	monotonic_zones mz = divide_to_monotonics(n);

	intersection_list il = {};
	for(auto z : mz.parts) {
		auto r = mz.inspect_zone(z, *this, l.a, n);
		if(r.id != -1) il.push_back(r);
	}
	if(il.size()%2) throw std::runtime_error("");
	return il;
}
point_cloud to_cloud(const poly& P, const intersection_list& L) {
	point_cloud cloud = {};
	for(auto I : L) {
		cloud.push_back(lerp(P[I.id], P[I.id + 1], I.t1));
	}
	return cloud;
}

// void draw_line(vec2 p, sf::RenderWindow& rw, sf::CircleShape& spr, box2 box) {
// 	spr.setPosition(box * p);
// 	rw.draw(spr);
// }