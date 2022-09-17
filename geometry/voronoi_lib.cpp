#include <cstdio>
#include <vector>
#include <boost/polygon/voronoi.hpp>
#include "utils.h"
#include "primitives.h"
#include "polygon.h"
#include "voronoi.h"
#include "spatial_graph.h"
#include <map>

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;

namespace boost {
	namespace polygon {
		template <>
		struct geometry_concept<vec2> {
			typedef point_concept type;
		};
		template <>
			struct point_traits<vec2> {
			typedef float coordinate_type;

			static inline coordinate_type get(const vec2& point, orientation_2d orient) {
				return (orient == HORIZONTAL) ? point.x : point.y;
			}
		};
		template <>
		struct geometry_concept<line> {
			typedef segment_concept type;
		};
		template <>
		struct segment_traits<line> {
			typedef float coordinate_type;
			typedef vec2 point_type;

			static inline point_type get(const line& segment, direction_1d dir) {
				return dir.to_int() ? segment.a : segment.b;
			}
		};
	}
}

spatial_graph make_voronoi_diagram_2(point_cloud sites) {
	const float M = 10000, Linf = 1000.f;
	for(auto& p : sites) p *= M;
	voronoi_diagram<double> vd;
	construct_voronoi(sites.begin(), sites.end(), &vd);
	spatial_graph result;
	u32 V = vd.vertices().size();
	// printf("V: %d\n", V);
	result.edges.resize(V);
	result.verts.resize(V);

	std::map<void*, u32> points_derefs;

	auto add_point = [&points_derefs, &result, M] (voronoi_diagram<double>::vertex_type* v) -> u32 {
		if(auto F = points_derefs.find(v); F == points_derefs.end()) {
			u32 s = points_derefs.size();
			points_derefs[v] = s;
			result.verts[s] = vec2(v->x(), v->y()) / M;
			return s;
		} else {
			return F->second;
		}
	};
	auto add_inf_point = [&sites, &result, Linf, M] (voronoi_diagram<double>::edge_type e) -> u32 {
		vec2 r = sites[e.cell()->source_index()] / M;
		vec2 l = sites[e.twin()->cell()->source_index()] / M;
		vec2 p = (r + l) * .5f - normalize(lrot(r - l)) * Linf;
		u32 s = result.verts.size();
		result.verts.push_back(p);
		result.edges.push_back({});
		return s;
	};
	for(auto e : vd.edges()) {
		auto va = e.vertex0(), vb = e.vertex1();
		// printf("va, vb: %x, %x\n", va, vb);
		if(va < vb) continue;
		u32 b;
		if(!vb) {
			// printf("b: inf\n");
			b = add_inf_point(e);
		} else {
			// printf("b: regular\n");
			b = add_point(e.vertex1());
		}
		u32 a = add_point(e.vertex0());
		// printf("a, b : %d, %d\n", a, b);
		result.edges[a].push_back(b); 
		result.edges[b].push_back(a); 
	}
	
	return result;
}
