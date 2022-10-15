#include "geometry.h"
#include <vector>
#include <map>

namespace geom {
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
	std::vector<Poly> scale_with_skeleton(const Poly& P, const skeleton_2& S, float h);
	std::vector<Poly> buffer(const Poly& P, float h);
}