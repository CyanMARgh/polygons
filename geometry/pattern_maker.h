#include "geometry.h"
#include "polygon.h"
#include "primitives.h"
#include <vector>
#include <queue>

struct Marked_Poly {
	Poly P;
	u32 type, depth;
};
struct Rule {
	enum Type {
		VORONOI,
		STRIPES,
		SQUARES, SQUARES_ROWS_2, SQUARES_ROWS_3, SQUARES_SHECKERBOARD, SQUARES_SUBQUADS_4,
		SQUARES_RANDOM_2, SQUARES_RANDOM_3, SQUARES_RANDOM_4,
		DISTRIBUTION,
		BUFFER,
		END
	} type;
	float size;
	u32 ids[4];
};
struct Placeable {
	Poly poly;
	Circle circle;
	static Placeable make(Poly P);
};

struct Pattern {
	std::vector<Rule> rules;
	u32 max_depth = 10;
	float min_area = 1.e-7;
	std::vector<Placeable> placeable_list = {};
};

void apply_rule(std::vector<Poly>& result, std::queue<Marked_Poly>& polys, const Pattern& pattern);

std::vector<Poly> process(const Pattern& pattern, Poly P);
