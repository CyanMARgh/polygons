#include "polygon.h"
#include <algorithm>
#include <numeric>
#include <set>
#include "primitives.h"
#include "utils.h"

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
intersection geom::inspect_zone( const poly& P, monotonic_zones::zone z, vec2 p, vec2 n) {
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
monotonic_zones geom::divide_to_monotonics(const poly& P, vec2 n) {
	if(!P.size) return {};

	std::vector<monotonic_zones::zone> parts;
	s32 i0 = 0;

	seg_type type0, type_temp;
	while((type0 = get_seg_type(P, i0, n)) == seg_type::ANY && i0 < P.size) i0++;
	if(i0 == P.size) return {{{0, (s32)P.size, type0}}};
	i0++;
	while(type_temp = get_seg_type(P, i0, n), (type_temp == seg_type::ANY || type_temp == type0) && i0 <= P.size) i0++;

	for(s32 i = i0, is = i0; i < i0 + P.size; i++) {
		seg_type type_next = get_seg_type(P, i + 1, n);
		if(type_next == seg_type::ANY) continue;
		if(type_temp != type_next) {
			parts.push_back({is, i + 1, type_temp});
			type_temp = type_next, is = i + 1;
		}
	}
	return {parts};
}
s32 geom::is_inside_val(const poly& P, const monotonic_zones& mz, vec2 p, vec2 n) {
	s32 s = 0;
	for(auto z : mz.parts) {
		auto r = geom::inspect_zone(P, z, p, n);
		if(r.id != -1) s += (r.t2 < 0) * (z.type == seg_type::UP ? 1 : -1);
	}
	return s;
}
seg_type geom::get_seg_type(const poly& P, s32 i, vec2 n) {
	float q = dot(n, P[i+1]-P[i]);
	return q > 0 ? seg_type::UP : q < 0 ? seg_type::DOWN : seg_type::ANY; 
}

//slice
intersection_list geom::find_intersections(const poly& P, line l) {
	if(l.b == l.a) return {};
	vec2 ab = l.b - l.a;
	vec2 n = rrot(ab);
	monotonic_zones mz = divide_to_monotonics(P, n);

	intersection_list il = {};
	for(auto z : mz.parts) {
		auto r = geom::inspect_zone(P, z, l.a, n);
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

std::vector<poly> divide(const poly& P, const intersection_list& L) {
	u32 n = L.size(), ps = P.size;
	if(!n) return {P};
	auto [ids, rids] = make_permutation<intersection>(L, [] (intersection a, intersection b) { return a.t2 <= b.t2; });

	std::vector<bool> used(n, false);

	auto next_c = [&] (u32 i) { return (i + 1) % n; };
	auto next_l = [&] (u32 i) { return  ids[rids[i] ^ 1]; };

	std::vector<poly> ans;
	for(u32 pi = 0; pi < n; pi++){
		if(used[pi]) continue;
		poly P0;
		for(u32 i = pi, j; !used[i]; ) {
			j = next_c(i);
			auto Is = L[i], Ie = L[j];
			u32 id_s = Is.id % ps, id_e = Ie.id % ps;
			P0.add(lerp(P[id_s], P[id_s + 1], Is.t1));
			for(auto k = id_s; k % ps != id_e; k++) { 
				P0.add(P[k + 1]);
			}
			P0.add(lerp(P[id_e], P[id_e + 1], Ie.t1));
			used[i] = true;
			i = next_l(j);
		}
		ans.push_back(P0);
	}

	return ans;
}

bool geom::has_self_intersections(const poly& P) {
	for(u32 n = P.size, i = 0; i < n; i++) {
		vec2 a = P[i], b = P[i + 1];
		for(u32 j = i + 2; j < n - !i; j++) {
			vec2 c = P[j], d = P[j + 1];
			if(check_intersection(a, b, c, d) != cross_type::NONE) return true;
		}
	}
	return false;

	// u32 n = P.size;
	// if(n < 4) return false;
	// auto [ids, rids] = make_permutation<vec2>(P.points, [] (vec2 a, vec2 b ) { return a.y < b.y; });

	// struct edge {
	// 	u32 a, b;
	// 	edge(u32 a, u32 b) : a(a), b(b) { }
	// 	bool operator<(const edge& o) const { return a == o.a ? b < o.b : a < o.a; }
	// };
	// auto intersects = [&P](edge e1, edge e2) { 
	// 	return e1.a != e2.b && e1.b != e2.a &&
	// 		check_intersection(P[e1.a], P[e1.b], P[e2.a], P[e2.b]) != cross_type::NONE;
	// };

	// std::set<edge> es = {};

	// for(u32 i = 0; i < n; i++) {
	// 	u32 id = ids[i];
	// 	u32 j0 = (id + n - 1) % n, j1 = (id + 1) % n; //nbs ids
	// 	edge e0 = {j0, id}, e1 = {id, j1};
	// 	bool c0 = i > rids[j0], c1 = i > rids[j1];
	// 	if(c0) {
	// 		es.erase(e0);
	// 	} else {
	// 		for(auto e : es) {
	// 			if(intersects(e, e0)) {
	// 				printf("{%d %d} x {%d %d}!\n", e.a, e.b, e0.a, e0.b);
	// 				return true;
	// 			}
	// 		}
	// 	}
	// 	if(c1) {
	// 		es.erase(e1); 
	// 	} else {
	// 		for(auto e : es) {
	// 			if(intersects(e, e1)) {
	// 				printf("{%d %d} x {%d %d}!\n", e.a, e.b, e1.a, e1.b);
	// 				return true;
	// 			}
	// 		}
	// 	}
	// 	if(!c0) es.insert(e0);
	// 	if(!c1) es.insert(e1);
	// }
	// return false;
}

bool geom::is_valid(const poly& P) {
	return area(P) > 0 && !has_self_intersections(P);
}