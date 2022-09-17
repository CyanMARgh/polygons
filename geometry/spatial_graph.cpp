#include "spatial_graph.h"
#include <algorithm>
#include "polygon.h"
#include <queue>
#include <map>
#include "utils.h"
#include "primitives.h"

std::set<id_line> add_self_intersections(spatial_graph& G, std::vector<id_line> marked) {
	struct event { u32 a, b; enum type_t { ADD, REMOVE } type; };
	struct intersection { u32 a, b, c, d; vec2 Q; };
	struct iedge {
		u32 a, b;
		bool operator<(const iedge& e2) const { return a == e2.a ? b < e2.b : a < e2.a; }
	};

	auto cmp_iv = [&G] (u32 Ai, u32 Bi) -> bool {
		vec2 A = G.verts[Ai], B = G.verts[Bi];
		return A.y == B.y ? A.x < B.x : A.y < B.y;
	};
	auto cmp_v = [&G] (vec2 A, vec2 B) -> bool {
		return A.y == B.y ? A.x < B.x : A.y < B.y;
	};
	auto event_cmp = [&cmp_iv] (event e1, event e2) -> bool {
		u32 p1 = e1.type == event::ADD ? e1.a : e1.b;
		u32 p2 = e2.type == event::ADD ? e2.a : e2.b;
		return cmp_iv(p1, p2);
	};

	std::priority_queue<event, std::vector<event>, decltype(event_cmp)> events(event_cmp);
	std::map<std::pair<u32, u32>, std::vector<u32>> replacements;
	std::vector<intersection> intersections;
	std::set<iedge> window = {};
	std::set<id_line> marked_fixed;

	auto current_edge = [&replacements] (u32 a, u32 b) -> std::pair<u32, u32> {
		if(auto F = replacements.find({a, b}); F != replacements.end()) {
			return {F->second.back(), b};
		} else {
			return {a, b};
		}
	};
	auto make_events = [&cmp_iv, &events, &G] () -> void {
		for(u32 i = 0, n = G.edges.size(); i < n; i++) {
			for(u32 j : G.edges[i]) {
				if(cmp_iv(i, j)) {
					events.push({i, j, event::ADD});
					events.push({i, j, event::REMOVE});
				}
			}
		}
	};
	auto add_intersection = [&intersections, &G] (u32 a, u32 b, u32 c, u32 d) -> void {
		if(a == c || b == c || a == d || b == d) return;
		vec2 A = G.verts[a], B = G.verts[b], C = G.verts[c], D = G.verts[d];
		cross_type T = check_intersection(A, B, C, D);
		if(T == cross_type::HAS) {
			vec2 Q = find_intersection(A, B, C, D);
			intersections.push_back({a, b, c, d, Q});
		}
	};
	auto apply_intersection = [&G, &current_edge, &replacements] (intersection I) -> void {
		u32 _a = I.a, _b = I.b, _c = I.c, _d = I.d;
		auto [a, b] = current_edge(_a, _b);
		auto [c, d] = current_edge(_c, _d);
		u32 q = G.verts.size();
		// printf("applying intersection: %d %d %d %d -> %d\n", _a, _b, _c, _d, q);
		auto add_replacement = [&replacements] (u32 a, u32 b, u32 q) -> void {
			if(auto F = replacements.find({a, b}); F != replacements.end()) {
				F->second.push_back(q);
			} else {
				replacements[{a, b}] = {q};
			}
		};
		add_replacement(_a, _b, q);
		add_replacement(_c, _d, q);
		auto reconnect = [&G] (u32 i, u32 jold, u32 jnew) {
			std::replace(G.edges[i].begin(), G.edges[i].end(), jold, jnew);
		};
		reconnect(a, b, q);
		reconnect(b, a, q);
		reconnect(c, d, q);
		reconnect(d, c, q);
		G.edges.push_back({a, b, c, d});
		G.verts.push_back(I.Q);
	};

	make_events();
	while(!events.empty()) {
		event e = events.top();
		events.pop();
		if(e.type == event::REMOVE) {
			window.erase({e.a, e.b});
		} else {
			for(auto ab : window) {
				add_intersection(ab.a, ab.b, e.a, e.b);
			}
			window.insert({e.a, e.b});
		}
	}
	std::sort(intersections.begin(), intersections.end(), [&cmp_v] (intersection i1, intersection i2) { return cmp_v(i1.Q, i2.Q); });
	for(auto I : intersections) { apply_intersection(I); }

	// printf("replacements:\n");
	// for(const auto& [k, v]: replacements) {
	// 	printf("[%d %d]:", k.first, k.second);
	// 	for(auto it: v) {
	// 		printf(" %d", it);
	// 	}
	// 	printf("\n");
	// }
	// printf("work with marked:\n");
	// auto print_marked_fixed = [&marked_fixed] {
	// 	printf("marked_fixed:\n");
	// 	for(auto l: marked_fixed) {
	// 		printf("{%d %d}\n", l.a, l.b);
	// 	}
	// };
	for(auto l: marked) {
		// printf("{%d %d}: ", l.a, l.b);
		if(auto F = replacements.find({l.a, l.b}); F != replacements.end()) {
			// printf("(0)\n");
			auto& ls = F->second;
			marked_fixed.insert({l.a, ls.front()});
			marked_fixed.insert({ls.back(), l.b});
			for(u32 i = 0, n = ls.size() - 1; i < n; i++) {
				marked_fixed.insert({ls[i], ls[i + 1]});
			}
		} else if(F = replacements.find({l.b, l.a}); F != replacements.end()) {
			// printf("(1)\n");
			auto& ls = F->second;
			marked_fixed.insert({l.a, ls.back()});
			marked_fixed.insert({ls.front(), l.b});
			for(u32 i = 0, n = ls.size() - 1; i < n; i++) {
				marked_fixed.insert({ls[i + 1], ls[i]});
			}
		} else {
			// printf("(2)\n");
			marked_fixed.insert(l);
		}
		// print_marked_fixed();
	}

	return marked_fixed;
}
// std::vector<poly> extract_inner_polygons(const spatial_graph& G) {
// 	return {};
// }

spatial_graph graph_union(const spatial_graph& A, const spatial_graph& B) {
	u32 Va = A.verts.size(), Vb = B.verts.size();
	spatial_graph R;
	R.edges.resize(Va + Vb), R.verts.resize(Va + Vb);
	std::copy(A.edges.begin(), A.edges.end(), R.edges.begin());
	std::copy(A.verts.begin(), A.verts.end(), R.verts.begin());
	std::copy(B.verts.begin(), B.verts.end(), R.verts.begin() + Va);
	for(u32 i = 0; i < Vb; i++) {
		u32 n = B.edges[i].size();
		std::vector<u32> nbs(n);
		for(u32 j = 0; j < n; j++) {
			nbs[j] = B.edges[i][j] + Va;
		}
		R.edges[i + Va] = nbs;		
	}
	return R;
}

spatial_graph spatial_graph::from_edge_list(const std::vector<id_line>& edges, std::vector<vec2> verts) {
	std::vector<std::vector<u32>> edges_2(verts.size());
	for(auto e : edges) {
		edges_2[e.a].push_back(e.b);
		edges_2[e.b].push_back(e.a);
	}
	return {edges_2, verts};
}

spatial_graph to_graph(poly P) {
	u32 n = P.size;
	spatial_graph G;
	G.verts = std::move(P.points);
	G.edges.resize(n);
	for(u32 i = 0; i < n; i++) {
		G.edges[i] = {(i + 1) % n, (i + n - 1) % n};
	}
	return G;
}
void print_graph(const spatial_graph& G) {
	for(u32 i = 0, n = G.edges.size(); i < n; i++) {
		printf("%d: { ", i);
		for(auto j : G.edges[i]) {
			printf("%d ", j);
		}
		printf("}\n");
	}
}

std::vector<poly> slice_poly(const poly& P, spatial_graph& G) {
	std::set<id_line> visited;
	std::vector<poly> result;
	std::vector<id_line> to_visit;

	auto next = [] (const std::vector<u32>& nbs, id_line l) -> id_line {
		u32 p = nbs.back();
		for(auto it : nbs) {
			if(it == l.a) return {l.b, p};
			p = it;
		}
		throw std::runtime_error("");
		return {-1u, -1u};
	};
	auto extract_poly = [&G, &result, &visited, &next, &to_visit] (id_line l0) -> void {
		// printf("exctracting...\n");
		poly Pi;
		id_line li = l0;
		do {
			// printf("(%d %d) ", li.a, li.b);
			if(visited.find(li) != visited.end()) {
				// printf("\nblacklist edge!\n");
				return;
			}
			Pi.add(G.verts[li.a]);
			visited.insert(li);
			to_visit.push_back({li.b, li.a}); //printf("added to stack: %d %d\n", li.b, li.a);
			li = next(G.edges[li.b], li);
		} while(li != l0);
		// printf("succesfully\n");
		result.push_back(Pi);
	};
	auto rearrange_edges = [&G] () -> void {
		for(u32 i = 0, n = G.verts.size(); i < n; i++) {
			vec2 A = G.verts[i];
			std::sort(G.edges[i].begin(), G.edges[i].end(), [&G, A] (u32 b1, u32 b2) {
				vec2 v1 = G.verts[b1] - A, v2 = G.verts[b2] - A;
				return atan2(v1.x, v1.y) < atan2(v2.x, v2.y);
			});
		}
	};


	{
		G = graph_union(to_graph(P), G);

		std::vector<id_line> visited_raw;
		for(u32 i = 0, n = P.size; i < n; i++) {
			visited_raw.push_back({(i + 1) % n, i});
		}
		visited = add_self_intersections(G, visited_raw);
		rearrange_edges();

		id_line visited0 = *(visited.begin());
		to_visit.push_back({visited0.b, visited0.a});
	}

	// printf("graph:\n");
	// print_graph(G);
	// printf("to_visit:\n");
	// for(auto l: to_visit) { printf("{%d, %d}\n", l.a, l.b); }
	// printf("blacklist:\n");
	// for(auto l: visited) { printf("{%d, %d}\n", l.a, l.b); }
	
	while(!to_visit.empty()) {
		id_line l = to_visit.back();
		to_visit.pop_back();
		extract_poly(l);
	}
	// printf("(4)\n");
	return result;
}




