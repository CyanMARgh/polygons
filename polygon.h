#pragma once
#include "box2.h"
#include <algorithm>

struct Line {
	vec2 a, b;
};

struct MonotonicZones;

enum class SegType {
	UP, DOWN, ANY
};

class Poly {
	std::vector<vec2> points;
	public:
	u32 size = 0;

	Poly() ;
	Poly(std::vector<vec2> _points);
	Poly& operator=(std::vector<vec2> _points);
	Poly(const Poly& poly);

	void add(vec2 p) ;
	vec2& operator[](s32 i);
	vec2 operator[](s32 i) const;

	void DrawPoly(sf::RenderWindow& rwin, Box2 box) const;
	float Area() const;
	vec2 AreaXCenter() const;
	vec2 MassCenter() const;
	MonotonicZones DivideToMonotonics() const;
	s32 IsInsideInt(const MonotonicZones& mz, vec2 p) const;
	SegType GetSegType(s32 i) const;
};

struct MonotonicZones {
	struct Zone {
		s32 a, b;
		SegType type;
	};
	std::vector<Zone> parts;
	static s32 InspectZone(Zone z, const Poly& poly, vec2 p);
	void print();
};

typedef std::vector<vec2> PointCloud;
struct ipoint {
	vec2 p;
	u32 id;
};
typedef std::vector<ipoint> IndexedCloud;

IndexedCloud ToSorted(const PointCloud& cloud);
std::vector<u32> MinimalHull(const IndexedCloud& cloud);
std::vector<u32> MinimalHull(const PointCloud& cloud);
Poly MakePoly(const PointCloud& cloud, const std::vector<u32>& ids);


Poly SortedPoly(const PointCloud& cloud);

bool VerifyMinimalHull(const std::vector<u32>& ids, const PointCloud& cloud);

