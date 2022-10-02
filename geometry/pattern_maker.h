#include "geometry.h"
#include "polygon.h"
#include "primitives.h"
#include <vector>
#include <queue>

struct Marked_Poly {
	Poly P;
	u32 type, depth;
	float area;
};
struct Rule {
	enum Type {
		VORONOI,
		STRIPES,
		SQUARES, SQUARES_ROWS_2, SQUARES_ROWS_3, SQUARES_SHECKERBOARD, SQUARES_SUBQUADS_4,
		SQUARES_RANDOM_2, SQUARES_RANDOM_3, SQUARES_RANDOM_4,
		DISTRIBUTION,
		BUFFER
	} type;
	float size;
	u32 ids[4];
};
struct Placeable {
	Poly poly;
	Circle circle;
	static Placeable make(Poly P) {
		// TODO
	}
};

struct Pattert {
	std::vector<Rule> rules;
	std::vector<Placeable> placeable_list;
	float min_area = 1.e-7;
	u32 max_depth = 10;
};

void apply_rule(std::queue<Marked_Poly>& polys, const Pattern& pattern) {
	Marked_Poly MP = polys.first(); polys.pop();
	Rule rule = pattern.rules[MP.type];
	switch (rule) {
		case VORONOI: {
			// TODO
			break;
		}
		// TODO
		default: {
			printf("unimplemented")
			break;
		}
	} 
	// TODO
}

std::vector<Poly> process(const Pattern& pattern, Poly P) {
	// TODO
}
