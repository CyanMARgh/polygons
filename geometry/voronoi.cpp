#include "voronoi.h"
#include "polygon.h"
#include "primitives.h"
#include "utils.h"
#include "spatial_graph.h"

std::vector<poly> divide_evenly(const poly& P, u32 guide, u32 iterations) {
	point_cloud sites; sites.resize(guide);
	box2 box = P.bounding_box();
	auto random_point = [&box] { return box * rand_vec2(); };
	for(auto& p : sites) p = random_point();
	// printf("(0)\n");
	std::vector<poly> result;
	for(u32 i = 0; i < iterations; i++) {
		// printf("(1)\n");
		if(i) {
			u32 n = result.size();
			for(u32 j = 0; j < n && j < guide; j++) sites[j] = geom::mass_center(result[j]);
			for(u32 j = n; j < guide; j++) sites[j] = random_point();
		}
		auto voronoi_graph = delaunay_to_voronoi(make_delaunay_triangulation(sites));
		result = slice_poly(P, voronoi_graph);
		// printf("(2)\n");
	}
	return result;
}

triangulation make_delaunay_triangulation(const point_cloud& sites_unsorted) {
	reindexed_cloud sites = geom::to_sorted_vertical(sites_unsorted);
	struct vert {
		u32 right, left;
	};
	
	std::map<id_line, id_line> quads = {};
	std::vector<vert> hull = {};

	auto print_hull = [&hull] () -> void {
		u32 i = 0;
		printf("[ ");
		do {
			printf("%u ", i);
			u32 j = hull[i].right;
			if(i == j) throw std::runtime_error("");
			i = j;
		} while (i);
		printf("]\n");
	};
	auto is_valid_quad = [](vec2 A, vec2 B, vec2 C, vec2 D) -> bool {
		// vec2 v1 = A - B, w1 = C - B, v2 = A - D, w2 = C - D;
		// return dot(v1, w1) * len(v2) * len(w2) > dot(v2, w2) * len(v1) * len(w1);
		vec2 v1 = normalize(A - B), w1 = normalize(C - B), v2 = normalize(A - D), w2 = normalize(C - D);
		float a1 = acos(dot(v1, w1)), a2 = M_PI - acos(dot(v2, w2));
		return a1 < a2;
	}; 
	auto is_valid_quad_i = [&is_valid_quad, &sites](u32 a, u32 b, u32 c, u32 d) -> bool {
		return is_valid_quad(sites.satat(a), sites.satat(b), sites.satat(c), sites.satat(d));
	}; 
	auto sort = [] (u32 x, u32 y) -> id_line { 
		if(x < y) {
			return {x, y};
		} else {
			return {y, x};
		}
	};
	auto flip_edge = [&quads, &sort] (u32 a, u32 b, u32 u, u32 d) {
		// printf("flip: a = %d, b = %d, d = %d, u = %d\n", a, b, d, u);
		auto update = [&quads, sort] (u32 x, u32 y, u32 iold, u32 inew) {
			// printf("update: [%d, %d]: %d -> %d\n", x, y, iold, inew);
			auto xy = sort(x, y);
			auto& ud = quads[xy];
			// printf("ud: %d, %d\n", ud.a, ud.b);
			if(ud.a == iold) {
				ud.a = inew;
			} else if (ud.b == iold) {
				ud.b = inew;
			} else {
				throw std::runtime_error("");
			}
		};
		auto ab = sort(a, b);
		update(a, u, b, d);
		update(a, d, b, u);
		update(d, b, a, u);
		update(b, u, a, d);
		quads.erase(ab);
		quads[sort(u, d)] = ab;
	};
	std::function<void(u32, u32, u32)> collapse;
	collapse = [&quads, &is_valid_quad_i, &sort, &collapse, &flip_edge] (u32 a, u32 b, u32 d) -> void {
		// printf("collapse: a = %d, b = %d, d = %d\n", a, b, d);
		auto get_second = [] (id_line xy, u32 x) -> u32 {
			// printf("get_second: xy.a = %d xy.b = %d, x = %d\n", xy.a, xy.b, x);
			return xy.a == x ? xy.b : xy.a;
		};
		u32 u = get_second(quads[sort(a, b)], d);
		if(u == -1u) return;
		// printf("\t\tu = %d\n", u);
		if(!is_valid_quad_i(a, u, b, d)) {
			// printf("(4)\n");
			flip_edge(a, b, u, d);
			// printf("(5)\n");
			collapse(a, u, d);
			// printf("(6)\n");
			collapse(u, b, d);
			// printf("(7)\n");
		} else {
			// printf("recursion stop\n");
		}
		// printf("(3)\n");
	};
	auto insert_site = [&print_hull, &collapse, &sort, &sites, &hull, &quads] (u32 k) {
		auto link_triangle = [&quads, &sort] (u32 i, u32 j, u32 k) -> void {
			auto link_partial = [&quads, &sort] (u32 i, u32 j, u32 k) -> void {
				// printf("linking: i=%d, j=%d, k=%d\n", i, j, k);
				auto ij = sort(i, j);
				auto F = quads.find(ij);
				if(F == quads.end()) {
					quads[ij] = {k, -1u};
				} else {
					auto& ab = F->second;
					// printf("quads[jk] = %d %d\n", ab.a, ab.b);
					if(ab.a == -1) {
						ab.a = k;
					} else if(ab.b == -1) {
						ab.b = k;
					} else {
						throw std::runtime_error("");
					}
				}
			};
			link_partial(i, j, k);
			link_partial(j, k, i);
			link_partial(k, i, j);
		};
		// print_hull();
		// printf("inserting:\n");
		u32 i = k - 1;
		vec2 Q = sites.satat(k), A = sites.satat(i);
		// printf("i = %d, k = %d.  Q = (%f, %f), A = (%f, %f)\n", i, k, Q.x, Q.y, A.x, A.y);
		while(i != 0) {
			u32 j = hull[i].right;
			// printf("<%d-%d>\n", i, j);
			vec2 B = sites.satat(j);
			if(cross(B - A, A - Q) > 0) {
				// printf("(0): %d %d\n", i, j);
				hull[i].right = hull[j].left = -1u;
				link_triangle(i, j, k);
				collapse(i, j, k);
			} else {
				break;
			}
			i = j, A = B;
		}
		u32 ir = i;
		// printf("i right = %d\n", i);
		// hull.push_back({i, -1u});
		// hull[i].left = k;
		i = k - 1;
		Q = sites.satat(k), A = sites.satat(i);
		// printf("i = %d, k = %d.  Q = (%f, %f), A = (%f, %f)\n", i, k, Q.x, Q.y, A.x, A.y);
		while(i != 0) {
			u32 j = hull[i].left;
			// printf("<%d-%d>\n", i, j);
			vec2 B = sites.satat(j);
			if(cross(B - A, A - Q) < 0) {				
				// printf("(1): %d %d\n", i, j);
				hull[i].left = hull[j].right = -1u;
				link_triangle(i, j, k);
				collapse(j, i, k);
			} else {
				break;
			}
			i = j, A = B;
		}
		u32 il = i;
		// printf("IL = %d, IR = %d\n", il, ir);
		hull.push_back({ir, il});
		hull[il].right = hull[ir].left = k;
		// printf("i left = %d\n", i);
		// hull[k].left = i;
		// hull[i].right = k;
	};
	auto init_triangle = [&quads, &sites, &hull] () {
		quads[{0, 1}] = {2, -1u};
		quads[{0, 2}] = {1, -1u};
		quads[{1, 2}] = {0, -1u};
		vec2 A = sites.satat(0), B = sites.satat(1), C = sites.satat(2);
		if(cross(B - A, C - A) < 0) {
			hull.push_back({2, 1});
			hull.push_back({0, 2});
			hull.push_back({1, 0});
		} else {
			hull.push_back({1, 2});
			hull.push_back({2, 0});
			hull.push_back({0, 1});
		}
	};
	triangulation result = {{}, &sites_unsorted};
	switch(sites.size()) {
		case 2:
			result.lines.push_back({{0, 1}, {-1u, -1u}});
		case 1:
			return result;
	}
	// printf("(0)\n");
	init_triangle();
	for(u32 k = 3, n = sites.size(); k < n; k++) {
		// printf("(2): %d\n", k);
		insert_site(k);
	}
	// printf("(======)\n");
	for(auto [k, v] : quads) {
		// printf("> %d %d %d %d\n", k.a, k.b, v.a, v.b);
		result.lines.push_back({{sites.at(k.a), sites.at(k.b)}, {v.a == -1u ? -1u : sites.at(v.a), v.b == -1u ? -1u : sites.at(v.b)}});
	}
	// printf("(======)\n");
	return result;
}

spatial_graph delaunay_to_voronoi(const triangulation& T) {
	const float length = 10000.f; 
	struct id_triangle {
		u32 a, b, c;
		bool operator<(id_triangle t2) const { return a == t2.a ? (b == t2.b ? c < t2.c : b < t2.b) : a < t2.a; }
	};
	const point_cloud* source = T.source;
	std::vector<vec2> verts;
	std::vector<id_line> edges;
	std::map<id_triangle, u32> tr_to_verts;

	auto find_center = [&source] (u32 a, u32 b, u32 c) -> vec2 {
		vec2 A = source->at(a), B = source->at(b), C = source->at(c);
		return circumcenter(A, B, C);
	};
	auto find_far = [&source, length] (u32 a, u32 b, u32 d) -> vec2 {
		vec2 A = source->at(a), B = source->at(b);
		vec2 AB = normalize(B - A);
		if(cross(source->at(d) - A, B - A) < 0) AB = -AB;
		return (A + B) * .5f + lrot(AB) * length;
	};
	auto sort = [] (id_triangle t) -> id_triangle {
		u32* tptr = (u32*)&t;
		std::sort(tptr, tptr + 3);
		return t;
	};
	auto add_point = [&sort, &find_far, &find_center, &tr_to_verts, &verts] (u32 a, u32 b, u32 c, u32 d) -> u32 {
		id_triangle t = sort({a, b, c});
		if(auto F = tr_to_verts.find(t); F == tr_to_verts.end()) {
			verts.push_back(c == -1u ? find_far(a, b, d) : find_center(a, b, c));
			return tr_to_verts[t] = verts.size() - 1;
		} else {
			return F->second;
		}
	};
	if(u32 s = T.lines.size(); !s) {
		return {};
	} else if(s == 1) {
		vec2 A = source->at(0), B = source->at(1);
		vec2 L = rrot(normalize(B - A)) * length, C = (A + B) * .5f;
		return {{{1},{0}}, {C + L, C - L}};
	}
	for(auto [l1, l2] : T.lines) {
		if(l2.a == -1u && l2.b != -1u) throw std::runtime_error("");
		u32 p1 = add_point(l1.a, l1.b, l2.b, l2.a);
		u32 p2 = add_point(l1.b, l1.a, l2.a, l2.b);
		edges.push_back({p1, p2});
	}
	return spatial_graph::from_edge_list(edges, verts);
}