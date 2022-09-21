#pragma once

#include <fstream>
#include <vector>
#include "geometry.h"
#include <map>

struct Intersection_List : std::vector<Intersection> {};
enum class Seg_Type {
	UP, DOWN, ANY
};

struct Poly {
	std::vector<vec2> points;
	u32 size = 0;

	Poly();
	Poly(std::vector<vec2> other);
	Poly& operator=(std::vector<vec2> other);
	Poly(const Poly& P);

	void add(vec2 p) ;
	vec2& operator[](s32 i);
	vec2 operator[](s32 i) const;
	Box2 bounding_box() const;
};
struct Monotonic_Zones {
	struct zone {
		s32 a, b;
		Seg_Type type;
	};
	std::vector<zone> parts;
};
struct Point_Cloud : std::vector<vec2> { };
struct Reindexed_Cloud : std::vector<u32> {
	const Point_Cloud* source;

	Reindexed_Cloud(std::vector<u32> ids = {}, const Point_Cloud* source = nullptr);

	Poly make_poly() const;

	vec2 satat(u32 i) const;
	vec2 sat(u32 i) const;
};

Point_Cloud to_cloud(const Poly& P, const Intersection_List& L);
std::vector<Poly> divide(const Poly& P, const Intersection_List& L);

struct Id_Line {
	u32 a, b;
	bool operator<(Id_Line l2) const;
	bool operator==(Id_Line l2) const;
	bool operator!=(Id_Line l2) const;
};
namespace geom {
	//mass
	float area(const Poly& P);
	vec2 area_X_center(const Poly& P);
	vec2 mass_center(const Poly& P);

	//monotonics
	Monotonic_Zones divide_to_monotonics(const Poly& P, vec2 n = {0, 1});
	Seg_Type get_seg_type(const Poly& P, s32 i, vec2 n = {0, 1});

	Intersection inspect_zone(const Poly& P, Monotonic_Zones::zone z, vec2 p, vec2 n = {0, 1});
	Intersection_List find_intersections(const Poly& P, Line l);
	s32 is_inside_val(const Poly& P, const Monotonic_Zones& mz, vec2 p, vec2 n = {0, 1});
	std::pair<std::vector<Poly>, Point_Cloud> divide(const Poly& P, vec2 p0, vec2 n, float h, u32 N);
	std::vector<Poly> divide_evenly(const Poly& P, Line L, float h);
	
	//welzl
	Circle welzl_2(vec2 a, vec2 b);
	Circle welzl_3(vec2 a, vec2 b, vec2 c);
	Circle welzl_trivial(const Reindexed_Cloud& rng);
	Circle welzl(Reindexed_Cloud P, Reindexed_Cloud R);
	Circle welzl(Point_Cloud cloud);

	//?
	bool has_self_intersections(const Poly& P);
	bool is_valid(const Poly& P);

	struct invalid_read : public std::invalid_argument {
		invalid_read(const std::string& filename);
	};

	//serialization
	std::ofstream& operator<<(std::ofstream& fout, const Poly& P);
	std::ifstream& operator>>(std::ifstream& fin, Poly& P);

	std::pair<float, vec2> bis_inter(Line X, Line Y, Line Z);
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

	Reindexed_Cloud to_sorted_vertical(const Point_Cloud&);
	skeleton make_skeleton_from_convex(const Poly& P);
	skeleton_2 make_skeleton_from_convex_2(const Poly& P);
	std::vector<Poly> scale_with_sceleton(const Poly& P, const skeleton_2& S, float h);
	std::vector<Poly> buffer(const Poly& P, float h);
}


// struct graph {
// 	struct edge {
// 		u32 a, b;
// 	};
// 	struct Intersection {
// 		edge e0, e1;
// 	};
// 	std::vector<vec2> vts;
// 	std::vector<edge> edges;

// 	static graph from_poly(const Poly& P);
// 	std::vector<Intersection> self_intersections() const;
// 	void add_intersections(const std::vector<Intersection>& inters);
// 	std::vector<Poly> extract_poly() const;
// };