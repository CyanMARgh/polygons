#include "geometry.h"

namespace geom {
	//hull
	//TODO REMAKE TO ONE METHOD WITH LAMBDA
	reindexed_cloud to_sorted(const point_cloud&);
	reindexed_cloud minimal_hull(const point_cloud&);
	reindexed_cloud to_circular_sorted(const point_cloud&);
	reindexed_cloud hull_by_circular(const reindexed_cloud&);
	bool is_right_convex(const poly& P);

	// TODO add minimal_hull_circular
	reindexed_cloud minimal_hull(const reindexed_cloud&);
	std::pair<bool, poly> verify_minimal_hull(const reindexed_cloud& cloud);
	std::tuple<bool, point_cloud, poly> minimal_hull_test(u32 N);
}