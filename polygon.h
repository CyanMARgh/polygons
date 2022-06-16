#include "utils.h"


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

	void DrawPoly(sf::RenderWindow& rwin) const;
	//void DrawPoly(sf::RenderWindow& rwin, const Extremes& extrs) const;

	float Area() const;
	vec2 AreaXCenter() const;
	vec2 MassCenter() const;
	MonotonicZones DivideToMonotonics() const;
	s32 IsInsideInt(const MonotonicZones& mz, vec2 p) const;
	SegType GetSegType(s32 i) const {
		float y0 = (*this)[i].y, y1 = (*this)[i + 1].y;
		return y0 < y1 ? SegType::UP : y0 > y1 ? SegType::DOWN : SegType::ANY; 
	}
};

struct MonotonicZones {
	struct Zone {
		s32 a, b;
		SegType type;
	};
	std::vector<Zone> parts;
	static s32 InspectZone(Zone z, const Poly& poly, vec2 p);
	void print() {
		for(auto z : parts) {
			printf("[%d,%d] ", z.a, z.b);
		}
		printf("\n");
	}
};
