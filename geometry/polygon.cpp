#include "polygon.h"
#include <algorithm>
#include <numeric>
#include <set>
#include "primitives.h"
#include "utils.h"
#include <queue>

//base
Poly::Poly() {
	points = {}, size = 0;
}
Poly::Poly(std::vector<vec2> other) : points(std::move(other)) { 
	size = points.size();
}
Poly& Poly::operator=(std::vector<vec2> other) {
	size = other.size();
	points = std::move(other);
	return *this;
}
Poly::Poly(const Poly& P) {
	points = P.points;
	size = P.size;
}
void Poly::add(vec2 p) {
	points.push_back(p);
	++size;
}
vec2& Poly::operator[](s32 i) {
	return points[mmod(i, points.size())];
}
vec2 Poly::operator[](s32 i) const {
	return points[mmod(i, points.size())];
}
Box2 Poly::bounding_box() const {
	vec2 mi = {1.e10f, 1.e10f}, ma = {-1.e10f, -1.e10f};
	for(auto p : points) {
		if(mi.x > p.x) mi.x = p.x;
		if(mi.y > p.y) mi.y = p.y;
		if(ma.x < p.x) ma.x = p.x;
		if(ma.y < p.y) ma.y = p.y;
	}
	return Box2(ma - mi, mi);
}

// is inside
Intersection geom::inspect_zone( const Poly& P, Monotonic_Zones::zone z, vec2 p, vec2 n) {
	float h0 = dot(p, n);
	float ay = dot(P[z.a], n), by = dot(P[z.b], n), cy;
	float mi = fmin(ay, by), ma = fmax(ay, by);

	if(z.type == Seg_Type::ANY) {
		printf("Seg_Type = ANY\n");
		return {0.f, 0.f, -1};
	}
	if(h0 <= mi || h0 >= ma) {
		// printf("ignored segment at %f\n", h0);
		return {0.f, 0.f, -1};
	}
	bool u_d = z.type == Seg_Type::UP;

	for(u32 c;;) {
		c = (z.a + z.b) / 2;
		cy = dot(P[c], n);
		if(z.a == c) break;
		((h0 < cy) ^ u_d ? z.a : z.b) = c;
	}

	vec2 A = P[z.a], B = P[z.b];
	float t1 = rlerp(dot(A, n), dot(B, n), h0);
	float t2 = lerp(cross(A, n), cross(B, n), t1) - h0;

	return {t1, t2, z.a};	
}
Monotonic_Zones geom::divide_to_monotonics(const Poly& P, vec2 n) {
	if(!P.size) return {};

	std::vector<Monotonic_Zones::zone> parts;
	s32 i0 = 0;

	Seg_Type type0, type_temp;
	while((type0 = get_seg_type(P, i0, n)) == Seg_Type::ANY && i0 < P.size) i0++;
	if(i0 == P.size) return {{{0, (s32)P.size, type0}}};
	i0++;
	while(type_temp = get_seg_type(P, i0, n), (type_temp == Seg_Type::ANY || type_temp == type0) && i0 <= P.size) i0++;

	for(s32 i = i0, is = i0; i < i0 + P.size; i++) {
		Seg_Type type_next = get_seg_type(P, i + 1, n);
		if(type_next == Seg_Type::ANY) continue;
		if(type_temp != type_next) {
			parts.push_back({is, i + 1, type_temp});
			type_temp = type_next, is = i + 1;
		}
	}
	return {parts};
}
s32 geom::is_inside_val(const Poly& P, const Monotonic_Zones& mz, vec2 p, vec2 n) {
	s32 s = 0;
	for(auto z : mz.parts) {
		auto r = geom::inspect_zone(P, z, p, n);
		if(r.id != -1) s += (r.t2 < 0) * (z.type == Seg_Type::UP ? 1 : -1);
	}
	return s;
}
Seg_Type geom::get_seg_type(const Poly& P, s32 i, vec2 n) {
	float q = dot(n, P[i+1]-P[i]);
	return q > 0 ? Seg_Type::UP : q < 0 ? Seg_Type::DOWN : Seg_Type::ANY; 
}

//slice
Intersection_List geom::find_intersections(const Poly& P, Line l) {
	if(l.b == l.a) return {};
	vec2 ab = l.b - l.a;
	vec2 n = rrot(ab);
	Monotonic_Zones mz = divide_to_monotonics(P, n);

	Intersection_List il = {};
	for(auto z : mz.parts) {
		auto r = geom::inspect_zone(P, z, l.a, n);
		if(r.id != -1) il.push_back(r);
	}
	if(il.size()%2) throw std::runtime_error("");

	return il;
}
Point_Cloud to_cloud(const Poly& P, const Intersection_List& L) {
	Point_Cloud cloud = {};
	for(auto I : L) {
		cloud.push_back(lerp(P[I.id], P[I.id + 1], I.t1));
	}
	return cloud;
}

std::vector<Poly> divide(const Poly& P, const Intersection_List& L) {
	u32 n = L.size(), ps = P.size;
	if(!n) return {P};
	auto [ids, rids] = make_permutation<Intersection>(L, [] (Intersection a, Intersection b) { return a.t2 <= b.t2; });

	std::vector<bool> used(n, false);

	auto next_c = [&] (u32 i) { return (i + 1) % n; };
	auto next_l = [&] (u32 i) { return  ids[rids[i] ^ 1]; };

	std::vector<Poly> ans;
	for(u32 pi = 0; pi < n; pi++) {
		if(used[pi]) continue;
		Poly P0;
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
std::vector<std::pair<Poly, u32>> geom::divide_to_stripes(const Poly& P, vec2 p0, vec2 n, float h, u32 N) {
	u32 ps = P.size;
	struct intersection_2 {
		u32 row, cid;
		float t1, t2; 
		bool is_down;
		u32 lid;
	};
	std::vector<intersection_2> il = {};
	auto inter_coord = [&P] (intersection_2 i) -> vec2 { return lerp(P[i.cid], P[i.cid + 1], i.t1); };

	Monotonic_Zones mz = divide_to_monotonics(P, n);
	for(u32 row = 0; row < N; row++) {
		for(u32 mzi = 0, mzs = mz.parts.size(); mzi < mzs; mzi++) {
			auto z = mz.parts[mzi];
			auto r = geom::inspect_zone(P, z, p0 - row * h * n, n);
			if(r.id != -1) {
				intersection_2 I = {row, (u32)r.id % P.size, r.t1, r.t2, z.type == Seg_Type::DOWN, row };
				il.push_back(I);
			}
		}
	}

	if(!il.size()) return {{P, 0}};
	u32 iln = il.size();
	std::sort(il.begin(), il.end(), [] (intersection_2 a, intersection_2 b) { 
		return a.row == b.row ? a.t2 <= b.t2 : a.row < b.row;
	});
	auto [ids, rids] = make_permutation<intersection_2>(il, [] (intersection_2 a, intersection_2 b) { 
		return a.cid == b.cid ? 
		a.is_down ^ (a.row > b.row) :
		a.cid < b.cid;
	});

	auto next_c = [&ids, &rids, iln] (u32 i) -> u32 { return ids[(rids[i] + 1) % iln]; };
	auto next_l = [] (u32 i) -> u32 { return i ^ 1; };

	std::vector<bool> visited(iln, false);
	std::vector<std::pair<Poly, u32>> result;

	for(u32 i = 0; i < iln; i++) {
		if(visited[i]) continue;
		Poly Pi;
		u32 row;
		for(u32 j = i, k; !visited[j]; ) {
			k = next_c(j);
			auto Is = il[j], Ie = il[k];
			u32 id_s = Is.cid, id_e = Ie.cid;
			if(i == j) row = Is.lid + Is.is_down;
			Pi.add(inter_coord(Is));
			for(auto l = id_s; l % ps != id_e; l++) { Pi.add(P[l + 1]); }
			Pi.add(inter_coord(Ie));
			visited[j] = true;
			j = next_l(k);
		}
		result.push_back({Pi, row});
	}
	return result;
}
std::vector<std::pair<Poly, u32>> geom::divide_to_stripes(const Poly& P, vec2 n, float h) {
	float t = -1.e10f, d = 1.e10f;
	for(auto p : P.points) {
		float my = dot(p, n);
		if(my > t) t = my;
		if(my < d) d = my;
	}
	s32 N = (s32)((t - d) / h) + 1; if(N < 0) N = 0;
	return geom::divide_to_stripes(P, (d + h * .5f) * n, -n, h, N);
}
std::vector<std::tuple<Poly, u32, u32>> geom::divide_to_squares(const Poly& P, vec2 n, float h) {
	float l = 1.e10f, r = -l, t = r, d = l;
	vec2 nr = rrot(n);
	for(auto p : P.points) {
		float mx = dot(p, nr), my = dot(p, n);
		if(mx > r) r = mx;
		if(mx < l) l = mx;
		if(my > t) t = my;
		if(my < d) d = my;
	}
	s32 N = (s32)((t - d) / h) + 1; if(N < 0) N = 0;
	s32 N2 = (s32)((r - l) / h) + 1; if(N2 < 0) N2 = 0;
	vec2 p0 = (d + h * .5f) * n + (l + h * .5f) * nr;

	std::vector<std::pair<Poly, u32>> hor_sliced = divide_to_stripes(P, p0, -n, h, N);
	std::vector<std::vector<std::pair<Poly, u32>>> ver_sliced = {};
	std::vector<std::tuple<Poly, u32, u32>> result;

	for(auto &Pi : hor_sliced) {
		auto vsi = divide_to_stripes(Pi.first, p0, -nr, h, N2);
		for(auto &Pij : vsi) {
			result.push_back({std::move(Pij.first), Pij.second, Pi.second});
		}
	}
	return result; 
}


bool geom::has_self_intersections(const Poly& P) {
	// for(u32 n = P.size, i = 0; i < n; i++) {
	// 	vec2 a = P[i], b = P[i + 1];
	// 	for(u32 j = i + 2; j < n - !i; j++) {
	// 		vec2 c = P[j], d = P[j + 1];
	// 		if(check_intersection(a, b, c, d) != cross_type::NONE) return true;
	// 	}
	// }
	// return false;

	u32 n = P.size;
	if(n < 4) return false;
	auto [ids, rids] = make_permutation<vec2>(P.points, [] (vec2 a, vec2 b ) { return a.y < b.y; });

	struct edge {
		u32 a, b;
		edge(u32 a, u32 b) : a(a), b(b) { }
		bool operator<(const edge& o) const { return a == o.a ? b < o.b : a < o.a; }
	};
	auto intersects = [&P](edge e1, edge e2) { 
		return e1.a != e2.b && e1.b != e2.a &&
			check_intersection(P[e1.a], P[e1.b], P[e2.a], P[e2.b]) != cross_type::NONE;
	};

	std::set<edge> es = {};

	for(u32 i = 0; i < n; i++) {
		u32 id = ids[i];
		u32 j0 = (id + n - 1) % n, j1 = (id + 1) % n; //nbs ids
		edge e0 = {j0, id}, e1 = {id, j1};
		bool c0 = i > rids[j0], c1 = i > rids[j1];
		if(c0) {
			es.erase(e0);
		} else {
			for(auto e : es) {
				if(intersects(e, e0)) {
					printf("{%d %d} x {%d %d}!\n", e.a, e.b, e0.a, e0.b);
					return true;
				}
			}
		}
		if(c1) {
			es.erase(e1); 
		} else {
			for(auto e : es) {
				if(intersects(e, e1)) {
					printf("{%d %d} x {%d %d}!\n", e.a, e.b, e1.a, e1.b);
					return true;
				}
			}
		}
		if(!c0) es.insert(e0);
		if(!c1) es.insert(e1);
	}
	return false;
}

bool geom::is_valid(const Poly& P) {
	return area(P) > 0 && !has_self_intersections(P);
}

// delaunay Triangulation & voronoi
bool Id_Line::operator<(Id_Line l2) const {
	return a == l2.a ? b < l2.b : a < l2.a;
}
bool Id_Line::operator==(Id_Line l2) const {
	return a == l2.a && b == l2.b;
}
bool Id_Line::operator!=(Id_Line l2) const {
	return a != l2.a || b != l2.b;
}
