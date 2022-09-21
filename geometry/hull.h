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
}