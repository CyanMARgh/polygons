#pragma once

#include <fstream>
#include <vector>
#include "geometry.h"
#include <map>

struct intersection_list : std::vector<intersection> {};
enum class seg_type {
	UP, DOWN, ANY
};

struct poly {
	std::vector<vec2> points;
	u32 size = 0;

	poly();
	poly(std::vector<vec2> other);
	poly& operator=(std::vector<vec2> other);
	poly(const poly& P);

	void add(vec2 p) ;
	vec2& operator[](s32 i);
	vec2 operator[](s32 i) const;
};
struct monotonic_zones {
	struct zone {
		s32 a, b;
		seg_type type;
	};
	std::vector<zone> parts;
};
struct point_cloud : std::vector<vec2> { };
struct reindexed_cloud : std::vector<u32> {
	const point_cloud* source;

	reindexed_cloud(std::vector<u32> ids = {}, const point_cloud* source = nullptr);

	poly make_poly() const;

	vec2 satat(u32 i) const;
	vec2 sat(u32 i) const;
};

point_cloud to_cloud(const poly& P, const intersection_list& L);
std::vector<poly> divide(const poly& P, const intersection_list& L);

namespace geom {
//	std::vector<poly>
	//mass
	float area(const poly& P);
	vec2 area_X_center(const poly& P);
	vec2 mass_center(const poly& P);

	//monotonics
	monotonic_zones divide_to_monotonics(const poly& P, vec2 n = {0, 1});
	seg_type get_seg_type(const poly& P, s32 i, vec2 n = {0, 1});

	intersection inspect_zone(const poly& P, monotonic_zones::zone z, vec2 p, vec2 n = {0, 1});
	intersection_list find_intersections(const poly& P, line l);
	s32 is_inside_val(const poly& P, const monotonic_zones& mz, vec2 p, vec2 n = {0, 1});
	std::pair<std::vector<poly>, point_cloud> divide(const poly& P, vec2 p0, vec2 n, float h, u32 N);
	std::vector<poly> divide_evenly(const poly& P, line L, float h);

	//hull
	//TODO REMAKE TO ONE METHOD WITH LAMBDA
	reindexed_cloud to_sorted(const point_cloud&);
	reindexed_cloud to_sorted_vertical(const point_cloud&);
	reindexed_cloud minimal_hull(const point_cloud&);
	reindexed_cloud to_circular_sorted(const point_cloud&);
	reindexed_cloud hull_by_circular(const reindexed_cloud&);
	bool is_right_convex(const poly& P);

	//			add minimal_hull_circular
	reindexed_cloud minimal_hull(const reindexed_cloud&);
	std::pair<bool, poly> verify_minimal_hull(const reindexed_cloud& cloud);
	std::tuple<bool, point_cloud, poly> minimal_hull_test(u32 N);

	//welzl
	circle welzl_2(vec2 a, vec2 b);
	circle welzl_3(vec2 a, vec2 b, vec2 c);
	circle welzl_trivial(const reindexed_cloud& rng);
	circle welzl(reindexed_cloud P, reindexed_cloud R);
	circle welzl(point_cloud cloud);

	//?
	bool has_self_intersections(const poly& P);
	bool is_valid(const poly& P);

	struct invalid_read : public std::invalid_argument {
		invalid_read(const std::string& filename);
	};

	//serialization
	std::ofstream& operator<<(std::ofstream& fout, const poly& P);
	std::ifstream& operator>>(std::ifstream& fin, poly& P);

	std::pair<float, vec2> bis_inter(line X, line Y, line Z);
	struct sk_event {
		float h;
		vec2 Q;
		u32 edge_A, edge_B, edge_C;
		u32 left_vert, right_vert, third_vert;

		enum type_t { EDGE, SPLIT, END } type;

		bool operator<(const sk_event& e2) const;
	};
	struct sk_event_2 {
		float h;
		vec2 Q;
		u32 iv_l, iv_r, iv_e;
		enum type_t { EDGE, SPLIT, END } type = EDGE;

		bool operator<(const sk_event_2& e2) const;
	};

	struct skeleton : std::vector<sk_event> { };
	struct skeleton_2 {
		std::vector<sk_event_2> events;
		std::map<u32, u32> iv_to_iske;
	};

	skeleton make_skeleton_from_convex(const poly& P);
	skeleton_2 make_skeleton_from_convex_2(const poly& P);
	std::vector<poly> scale_with_sceleton(const poly& P, const skeleton_2& S, float h);
	std::vector<poly> buffer(const poly& P, float h);

	struct id_line { u32 a, b; };
	struct triangulation {
		std::vector<id_line> lines; 
		const point_cloud* source;
	};
	triangulation make_delaunay_triangulation(const point_cloud& sites_unsorted);
}


// struct graph {
// 	struct edge {
// 		u32 a, b;
// 	};
// 	struct intersection {
// 		edge e0, e1;
// 	};
// 	std::vector<vec2> vts;
// 	std::vector<edge> edges;

// 	static graph from_poly(const poly& P);
// 	std::vector<intersection> self_intersections() const;
// 	void add_intersections(const std::vector<intersection>& inters);
// 	std::vector<poly> extract_poly() const;
// };