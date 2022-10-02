#pragma once

#include <fstream>
#include <vector>
#include "geometry.h"
#include <map>

struct Intersection_List : std::vector<Intersection> {};
enum class Seg_Type {
	UP, DOWN, ANY
};

struct Poly {
	std::vector<vec2> points;
	u32 size = 0;

	Poly();
	Poly(std::vector<vec2> other);
	Poly& operator=(std::vector<vec2> other);
	Poly(const Poly& P);

	void add(vec2 p) ;
	vec2& operator[](s32 i);
	vec2 operator[](s32 i) const;
	Box2 bounding_box() const;
};
struct Monotonic_Zones {
	struct zone {
		s32 a, b;
		Seg_Type type;
	};
	std::vector<zone> parts;
};
struct Point_Cloud : std::vector<vec2> { };
struct Reindexed_Cloud : std::vector<u32> {
	const Point_Cloud* source;

	Reindexed_Cloud(std::vector<u32> ids = {}, const Point_Cloud* source = nullptr);

	Poly make_poly() const;

	vec2 satat(u32 i) const;
	vec2 sat(u32 i) const;
};

Point_Cloud to_cloud(const Poly& P, const Intersection_List& L);
std::vector<Poly> divide(const Poly& P, const Intersection_List& L);

struct Id_Line {
	u32 a, b;
	bool operator<(Id_Line l2) const;
	bool operator==(Id_Line l2) const;
	bool operator!=(Id_Line l2) const;
};
namespace geom {
	//mass
	float area(const Poly& P);
	vec2 area_X_center(const Poly& P);
	vec2 mass_center(const Poly& P);

	//monotonics
	Monotonic_Zones divide_to_monotonics(const Poly& P, vec2 n = {0, 1});
	Seg_Type get_seg_type(const Poly& P, s32 i, vec2 n = {0, 1});

	Intersection inspect_zone(const Poly& P, Monotonic_Zones::zone z, vec2 p, vec2 n = {0, 1});
	Intersection_List find_intersections(const Poly& P, Line l);
	s32 is_inside_val(const Poly& P, const Monotonic_Zones& mz, vec2 p, vec2 n = {0, 1});
	std::vector<std::pair<Poly, u32>> divide_to_stripes(const Poly& P, vec2 p0, vec2 n, float h, u32 N);
	std::vector<std::pair<Poly, u32>> divide_to_stripes(const Poly& P, Line L, float h);
	std::vector<std::tuple<Poly, u32, u32>> divide_to_squares(const Poly& P, Line L, float h);
	
	//?
	bool has_self_intersections(const Poly& P);
	bool is_valid(const Poly& P);

	struct invalid_read : public std::invalid_argument {
		invalid_read(const std::string& filename);
	};

	//serialization
	std::ofstream& operator<<(std::ofstream& fout, const Poly& P);
	std::ifstream& operator>>(std::ifstream& fin, Poly& P);
}
