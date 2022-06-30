#pragma once
#include <vector>
#include "utils.h"
#include "transforms.h"

struct line {
	vec2 a, b;
};

struct monotonic_zones;
struct reindexed_cloud;

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

	void draw(sf::RenderWindow& rwin, box2 box) const;
	float area() const;
	vec2 area_X_center() const;
	vec2 mass_center() const;
	//monotonic_zones divide_to_monotonics() const;
	monotonic_zones divide_to_monotonics(vec2 n = {0, 1}) const;
	s32 is_inside_val(const monotonic_zones& mz, vec2 p) const;
	seg_type get_seg_type(s32 i, vec2 n) const;
};

struct intersection {
	float t1, t2;
	s32 id;
};
typedef std::vector<intersection> intersection_list;

struct monotonic_zones {
	struct zone {
		s32 a, b;
		seg_type type;
	};
	std::vector<zone> parts;
	//static s32 inspect_zone(zone z, const poly& P, vec2 p);
	static s32 inspect_zone(zone z, const poly& P, vec2 p, vec2 n = {0, 1}); //!!!!!!!!

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
	vec2 c = {};
	float r2 = 0.f;

	bool inside(vec2 p) const;
	void draw(sf::RenderWindow& rwin, sf::CircleShape& spr, box2 box) const;
};

vec2 trivial3(vec2 a, vec2 b, vec2 c);
circle trivial(const reindexed_cloud& rng);
circle welzl(reindexed_cloud P, reindexed_cloud R);
circle welzl(point_cloud cloud);