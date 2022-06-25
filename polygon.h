#pragma once
#include <vector>
#include "utils.h"
#include "box2.h"

struct Line {
	vec2 a, b;
};

struct MonotonicZones;
struct ReindexedCloud;

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

struct PointCloud : std::vector<vec2> {
	ReindexedCloud ToSorted() const;
	ReindexedCloud ToCircularSorted() const;
	ReindexedCloud MinimalHull() const;
};

struct ReindexedCloud : std::vector<u32> {
	const PointCloud* source;

	ReindexedCloud(std::vector<u32> ids = {}, const PointCloud* source = nullptr);

	ReindexedCloud HullByCircular() const;
	ReindexedCloud MinimalHull() const;
	Poly MakePoly() const;

	std::pair<bool, Poly> VerifyMinimalHull() const;
	static std::tuple<bool, PointCloud, Poly> MinimalHullTest(u32 N);
};
