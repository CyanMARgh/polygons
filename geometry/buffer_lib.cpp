#include "polygon.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <iostream>
#include "utils.h"
#include "buffer.h"

std::vector<Poly> geom::buffer(const Poly& P, float h) {
	namespace bg = boost::geometry;
	s32 M = 100000;

	typedef bg::model::d2::point_xy<s32> b_vec2;
	typedef bg::model::polygon<b_vec2> b_Poly;
	typedef bg::model::multi_polygon<b_Poly> b_Multi_Poly;

	const int PPC = 1;
    bg::strategy::buffer::distance_symmetric<s32> distance_strategy(h * M);
    bg::strategy::buffer::join_round join_strategy(PPC);
    //bg::strategy::buffer::join_miter join_strategy;
    bg::strategy::buffer::end_round end_strategy(PPC);
    bg::strategy::buffer::point_circle circle_strategy(PPC);
    bg::strategy::buffer::side_straight side_strategy;

    b_Multi_Poly Q1, P3;
    std::vector<b_vec2> P1;
    for(auto p : P.points) { 
    	P1.push_back({(s32)(p.x * M), (s32)(p.y * M)});
    }
    P1.push_back(P1[0]);
    b_Poly P2;
    bg::assign_points(P2, P1);
    P3.push_back(P2);

	bg::buffer(P3, Q1,
		distance_strategy, side_strategy,
		join_strategy, end_strategy, circle_strategy
	);

	std::vector<Poly> Q2;
	for(auto Q1i : Q1) {
		bg::validity_failure_type failure;
		bool valid = bg::is_valid(Q1i, failure);
		if(!valid) continue;
		auto er = bg::exterior_ring(Q1i);
		Poly Q2i;
		for(u32 i = 0, n = er.size() - 1; i < n; i++) {
			b_vec2 v = er[i];
			Q2i.add({((float)v.x()) / M, ((float)v.y()) / M});
		}
		Q2.push_back(Q2i);
	}
	return Q2;
}