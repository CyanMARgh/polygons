#pragma once
#include <vector>
#include "utils.h"
#include "transforms.h"

struct line {
	vec2 a, b;
};

struct monotonic_zones;
struct reindexed_cloud;
struct intersection;
typedef std::vector<intersection> intersection_list;

enum class seg_type {
	UP, DOWN, ANY
};

class poly {
	std::vector<vec2> points;
	public:
	u32 size = 0;

	poly() ;
	poly(std::vector<vec2> other);
	poly& operator=(std::vector<vec2> other);
	poly(const poly& P);

	void add(vec2 p) ;
	vec2& operator[](s32 i);
	vec2 operator[](s32 i) const;

	float area() const;
	vec2 area_X_center() const;
	vec2 mass_center() const;

	monotonic_zones divide_to_monotonics(vec2 n = {0, 1}) const;
	intersection_list find_intersections(line l) const;

	s32 is_inside_val(const monotonic_zones& mz, vec2 p, vec2 n = {0, 1}) const;
	seg_type get_seg_type(s32 i, vec2 n) const;
};
// struct generic_poly : std::vector<poly> {

// };

struct intersection {
	float t1, t2;
	s32 id;
	//bool i_o;
};

struct monotonic_zones {
	struct zone {
		s32 a, b;
		seg_type type;
	};
	std::vector<zone> parts;
	static intersection inspect_zone(zone z, const poly& P, vec2 p, vec2 n = {0, 1});

	void print();
};

struct point_cloud : std::vector<vec2> {
	reindexed_cloud to_sorted() const;
	reindexed_cloud to_circular_sorted() const;
	reindexed_cloud minimal_hull() const;
};
struct reindexed_cloud : std::vector<u32> {
	const point_cloud* source;

	reindexed_cloud(std::vector<u32> ids = {}, const point_cloud* source = nullptr);

	reindexed_cloud hull_by_circular() const;
	reindexed_cloud minimal_hull() const;
	poly make_poly() const;

	std::pair<bool, poly> verify_minimal_hull() const;
	static std::tuple<bool, point_cloud, poly> minimal_hull_test(u32 N);

	vec2 satat(u32 i) const;
	vec2 sat(u32 i) const;
};
struct cloud_range {
	point_cloud* source;
	u32 a, b;

	s32 size() const; 
	vec2 at(u32 i) const;
};
struct circle {
	vec2 f1 = {};
	vec2 f2 = {};

	bool inside(vec2 p) const;
	void draw(sf::RenderWindow& rwin, sf::CircleShape& spr, box2 box) const;

	float rad() const;
	vec2 center() const;
};

namespace welzl {
	circle trivial2(vec2 a, vec2 b);
	circle trivial3(vec2 a, vec2 b, vec2 c);
	circle trivial(const reindexed_cloud& rng);
	circle get(reindexed_cloud P, reindexed_cloud R);
	circle get(point_cloud cloud);
}

point_cloud to_cloud(const poly& P, const intersection_list& L);
bool check_self_intersections(const poly& P);  //!!!!!!!!
std::vector<poly> divide(const poly& P, const intersection_list& L) {


struct plot {
	struct options {
		sf::RenderWindow* rw;
		box2 box;
		sf::CircleShape circle;

		options(sf::RenderWindow& rw);
		void draw_point(vec2 p);
		void draw_cloud(const point_cloud& cloud);
		void draw_cloud(const reindexed_cloud& cloud);
		void draw_poly(const poly& P);
	};
	static std::unique_ptr<options> opt;
	static void init(sf::RenderWindow& rw);

	plot(sf::RenderWindow& rw);
	// static void poly(const poly& P);
	// static void circle(circle c);
	options* operator->();
};