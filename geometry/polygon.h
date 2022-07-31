#pragma once

#include <fstream>
#include <vector>
#include "geometry.h"

struct intersection_list : std::vector<intersection> {};
enum class seg_type {
	UP, DOWN, ANY
};

struct poly {
	std::vector<vec2> points;
	u32 size = 0;

	poly() ;
	poly(std::vector<vec2> other);
	poly& operator=(std::vector<vec2> other);
	poly(const poly& P);

	void add(vec2 p) ;
	vec2& operator[](s32 i);
	vec2 operator[](s32 i) const;
};
struct monotonic_zones {
	struct zone {
		s32 a, b;
		seg_type type;
	};
	std::vector<zone> parts;
};
struct point_cloud : std::vector<vec2> { };
struct reindexed_cloud : std::vector<u32> {
	const point_cloud* source;

	reindexed_cloud(std::vector<u32> ids = {}, const point_cloud* source = nullptr);

	poly make_poly() const;

	vec2 satat(u32 i) const;
	vec2 sat(u32 i) const;
};

point_cloud to_cloud(const poly& P, const intersection_list& L);
std::vector<poly> divide(const poly& P, const intersection_list& L);

namespace geom {
	//mass
	float area(const poly& P);
	vec2 area_X_center(const poly& P);
	vec2 mass_center(const poly& P);

	//monotonics
	monotonic_zones divide_to_monotonics(const poly& P, vec2 n = {0, 1});
	seg_type get_seg_type(const poly& P, s32 i, vec2 n = {0, 1});

	intersection inspect_zone(const poly& P, monotonic_zones::zone z, vec2 p, vec2 n = {0, 1});
	intersection_list find_intersections(const poly& P, line l);
	s32 is_inside_val(const poly& P, const monotonic_zones& mz, vec2 p, vec2 n = {0, 1});

	//hull
	reindexed_cloud to_sorted(const point_cloud&);
	reindexed_cloud minimal_hull(const point_cloud&);
	reindexed_cloud to_circular_sorted(const point_cloud&);
	reindexed_cloud hull_by_circular(const reindexed_cloud&);
	bool is_right_curved(const poly& P);

	//			add minimal_hull_circular
	reindexed_cloud minimal_hull(const reindexed_cloud&);
	std::pair<bool, poly> verify_minimal_hull(const reindexed_cloud& cloud);
	std::tuple<bool, point_cloud, poly> minimal_hull_test(u32 N);

	//welzl
	circle welzl_2(vec2 a, vec2 b);
	circle welzl_3(vec2 a, vec2 b, vec2 c);
	circle welzl_trivial(const reindexed_cloud& rng);
	circle welzl(reindexed_cloud P, reindexed_cloud R);
	circle welzl(point_cloud cloud);

	//?
	bool has_self_intersections(const poly& P);
	bool is_valid(const poly& P);

	struct invalid_read : public std::invalid_argument {
		invalid_read(const std::string& filename);
	};

	//serialization
	std::ofstream& operator<<(std::ofstream& fout, const poly& P);
	std::ifstream& operator>>(std::ifstream& fin, poly& P);


	//template with vaarg

	// template<typename T>
	// void save_to_file(const T& data, const std::string& filename) {
	// 	std::ofstream fout(filename);
	// 	fout << data;
	// }
	// template<typename T>
	// void load_from_file(T& data, const std::string& filename) {
	// 	std::ifstream fin(filename);
	// 	fin >> data;
	// }

	//straight skeleton
	// struct circle_list {
	// 	struct node {
	// 		u32 data;
	// 		node* next = nullptr, *prev = nullptr;

	// 		node(u32 data) : data(data) { }
	// 	};
	// 	static node* next(node* N) {
	// 		return N ? N->next : nullptr;
	// 	}
	// 	static node* next(node* N) {
	// 		return N ? N->next : nullptr;
	// 	}
	// 	static void connect(node* Np, node* Nn) {
	// 		Np->next = Nn, Nn->prev = Np;
	// 	}

	// 	node* root = nullptr;
	// 	circle_list operator=(const circle_list&) = delete;
	// 	~circle_list() {
	// 		if(!root) return;
	// 		node R = root;
	// 		do {
	// 			R_ = R->next;
	// 			delete R;
	// 		} while(R_ != root);
	// 	}
	// 	void add(u32 val) {
	// 		if(root) {
	// 			node* N = new node(val);
	// 			connect(root->prev, N);
	// 			connect(N, root);
	// 		} else {
	// 			root = new node(val);
	// 		}
	// 	}
	// 	void remove(node* N) {
	// 		if(N == root) {
	// 			if(root == root->next) {
	// 				delete N;
	// 				root = nullptr;
	// 				return;
	// 			}
	// 			root = root->next;
	// 		}
	// 		connect(N->prev, N->next);
	// 		delete N;
	// 	}		
	// };
	//					collapse order

	std::pair<float, vec2> bis_inter(line X, line Y, line Z);
	struct skeleton : std::vector<u32> { };
	skeleton make_skeleton_from_convex(const poly& P);
	// 	//
	// 	// vector< skeleton::node >

	// 	struct event {
	// 		float depth;
	// 		node* 
	// 	};
	// }
}


// struct graph {
// 	struct edge {
// 		u32 a, b;
// 	};
// 	struct intersection {
// 		edge e0, e1;
// 	};
// 	std::vector<vec2> vts;
// 	std::vector<edge> edges;

// 	static graph from_poly(const poly& P);
// 	std::vector<intersection> self_intersections() const;
// 	void add_intersections(const std::vector<intersection>& inters);
// 	std::vector<poly> extract_poly() const;
// };