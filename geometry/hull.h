#include "geometry.h"

namespace geom {
	//hull
	//TODO REMAKE TO ONE METHOD WITH LAMBDA
	Reindexed_Cloud to_sorted(const Point_Cloud&);
	Reindexed_Cloud minimal_hull(const Point_Cloud&);
	Reindexed_Cloud to_circular_sorted(const Point_Cloud&);
	Reindexed_Cloud hull_by_circular(const Reindexed_Cloud&);
	bool is_right_convex(const Poly& P);

	// TODO add minimal_hull_circular
	Reindexed_Cloud minimal_hull(const Reindexed_Cloud&);
	std::pair<bool, Poly> verify_minimal_hull(const Reindexed_Cloud& cloud);
	std::tuple<bool, Point_Cloud, Poly> minimal_hull_test(u32 N);

	Circle welzl_2(vec2 a, vec2 b);
	Circle welzl_3(vec2 a, vec2 b, vec2 c);
	Circle welzl_trivial(const Reindexed_Cloud& rng);
	Circle welzl(Reindexed_Cloud P, Reindexed_Cloud R);
	Circle welzl(Point_Cloud cloud);
}