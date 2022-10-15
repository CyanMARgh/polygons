#include "pattern_maker.h"
#include "buffer.h"
#include "polygon.h"
#include "plot.h"
#include "utils.h"
#include "hull.h"
#include <cmath>
#include "spatial_graph.h"

Placeable Placeable::make(Poly P) {
	return {
		.poly = P,
		.circle = geom::welzl_poly(P)
	};
}

// VORONOI,
// STRIPES,
// SQUARES, SQUARES_ROWS_2, SQUARES_ROWS_3, SQUARES_SHECKERBOARD, SQUARES_SUBQUADS_4,
// SQUARES_RANDOM_2, SQUARES_RANDOM_3, SQUARES_RANDOM_4,
// DISTRIBUTION,
// BUFFER

Circle make_inner_circle(const Poly& P, vec2 O) {
	float r2 = 1.e10f;
	for(u32 i = 0; i < P.size; i++) {
		vec2 a = P[i], b = P[i + 1];
		float k = dot(O - a, b - a) / (len2(b - a) + 1.e-10f);
		k = k < 0.f ? 0.f : k > 1.f ? 1.f : k;
		float R2 = len2(lerp(a, b, k) - O);
		if(R2 < r2) r2 = R2;
	}
	return Circle::make(O, sqrtf(r2));
}
vec2 rotate(vec2 O1, vec2 O2, float r1, float r2, vec2 v, float phi) {
	v = (v - O1) * (r2 / r1);
	v = {v.x * cosf(phi) + v.y * sinf(phi), v.y * cosf(phi) - v.x * sinf(phi)};
	return v + O2;
}

Poly place_inside(const Poly& outer, vec2 O, const Placeable& placeable, float phi) {
	Circle Co = make_inner_circle(outer, O);
	Circle Ci = placeable.circle;
	float r1 = Ci.rad(), r2 = Co.rad();
	// printf("r1, r2 = %f, %f\n", r1, r2);
	vec2 O1 = Ci.center(), O2 = Co.center();
	Poly result;
	for(auto p : placeable.poly.points) {
		result.add(rotate(O1, O2, r1, r2, p, phi));
		// result.add((p - O1) * (r2 / r1) + O2);
	}
	return result;
}

void apply_rule(std::vector<Poly>& result, std::queue<Marked_Poly>& polys, const Pattern& pattern) {
	Marked_Poly MP = polys.front(); polys.pop();
	if(MP.depth >= pattern.max_depth) {
		printf("too deep\n");
		return;
	}
	float area = geom::area(MP.P); 
	if(area < pattern.min_area) {
		printf("too small (area = %12.10f, type = %d, p = %d)\n", area, MP.type, MP.P.size);
		return;
	}
	if(geom::has_self_intersections(MP.P)) {
		printf("broken poly, type = %d", MP.P.size);
		return;
	}
	result.push_back(MP.P);

	u32 new_depth = MP.depth + 1;
	Rule rule = pattern.rules[MP.type];

	auto shake = [] (Poly& P) {
		const float eps = 1.e-5f;
		for(auto& p : P.points) {
			vec2 delta = rand_vec2() * eps;
			// printf("(%f, %f) + (%f, %f)\n", p.x, p.y, delta.x, delta.y);
			p += delta;
		}
	};

	shake(MP.P);
	switch (rule.type) {
		case Rule::BUFFER: {
			// std::vector<Poly> buffered = geom::buffer(MP.P, -rule.size);

			auto sk = geom::make_skeleton_from_convex_2(MP.P);
			std::vector<Poly> buffered = geom::scale_with_skeleton(MP.P, sk, rule.size); 

			// printf("buffer: -> %ld\n", buffered.size());
			for(auto&& P : buffered) {
				polys.push({std::move(P), rule.ids[0], new_depth});
			}
			break;
		}
		case Rule::STRIPES: {
			std::vector<std::pair<Poly, u32>> stripes = geom::divide_to_stripes(MP.P, rand_unit2(), rule.size);
			// printf("stripes: -> %ld\n", stripes.size());
			for(auto&& P : stripes) {
				polys.push({std::move(P.first), rule.ids[0], new_depth});
			}
			break;
		}
		case Rule::SQUARES: {
			std::vector<std::tuple<Poly, u32, u32>> squares = geom::divide_to_squares(MP.P, rand_unit2(), rule.size);
			// printf("squares: -> %ld\n", squares.size());
			for(auto&& P : squares) {
				polys.push({std::move(std::get<0>(P)), rule.ids[0], new_depth});
			}
			break;
		}
		case Rule::SQUARES_SHECKERBOARD: {
			std::vector<std::tuple<Poly, u32, u32>> squares = geom::divide_to_squares(MP.P, rand_unit2(), rule.size);
			// printf("squares: -> %ld\n", squares.size());
			for(auto&& P : squares) {
				u32 I = std::get<1>(P) + std::get<2>(P);
				polys.push({std::move(std::get<0>(P)), rule.ids[I % 2], new_depth});
			}
			break;

		}
		case Rule::END: {
			break;
		}
		case Rule::VORONOI: {
			u32 N = (int)(area / (rule.size * rule.size)) + 1;
			auto zones = divide_evenly(MP.P, N, 30);
			//u32 n = std::min(zones.first.size(), zones.second.size());
			// printf("voronoi: -> %ld\n", zones.first.size());
			for(auto&& z : zones.first) {
				polys.push({std::move(z), rule.ids[0], new_depth});
			}
			break;
		}
		case Rule::DISTRIBUTION: {
			u32 N = (int)(area / (rule.size * rule.size)) + 1;
			auto zones = divide_evenly(MP.P, N, 30);
			const Placeable& placeable = pattern.placeable_list[rule.ids[1]];
			for(u32 i = 0, n = zones.first.size(); i < n; i++) {
				const auto& P = zones.first[i];
				const auto& O = zones.second[i];
				printf("area = %f, O = (%f, %f)\n", geom::area(P), O.x, O.y);
				polys.push({place_inside(P, O, placeable, randf() * 2.f * M_PI), rule.ids[0], new_depth});				
			}
			break;
		}
		// TODO
		default: {
			// printf("unimplemented");
			break;
		}
	} 
}

std::vector<Poly> process(const Pattern& pattern, Poly P) {
	std::vector<Poly> result;
	std::queue<Marked_Poly> polys;

	polys.push({std::move(P), 0, 0});
	while(!(polys.empty())) {
		// printf("queue size: %ld\n", polys.size());
		apply_rule(result, polys, pattern);
	}

	return result;
}
