#include "polygon.h"


Poly::Poly() {
	points = {}, size = 0;
}
Poly::Poly(std::vector<vec2> _points) : points(std::move(_points)) { 
	size = points.size();
}
Poly& Poly::operator=(std::vector<vec2> _points) {
	points = std::move(_points);
	size = _points.size();
	return *this;
}
Poly::Poly(const Poly& poly) {
	points = poly.points;
	size = poly.size;
}
void Poly::add(vec2 p) {
	points.push_back(p);
	++size;
}
vec2& Poly::operator[](s32 i) {
	return points[mmod(i, points.size())];
}
vec2 Poly::operator[](s32 i) const {
	return points[mmod(i, points.size())];
}

void Poly::DrawPoly(sf::RenderWindow& rwin) const {
	vec2 S = (vec2)rwin.getSize();
	if(size < 1) return;
	sf::Vertex *vlines = new sf::Vertex[size + 1];
	for(u32 i = 0; i <= size; i++) {
		vlines[i] = (*this)[i] * S;
	}

	rwin.draw(vlines, size + 1, sf::LineStrip);
	delete[] vlines;
}
/*void Poly::DrawPoly(sf::RenderWindow& rwin, const Extremes& extrs) const {
	sf::Color red = {255,0,0}, blue = {0,0,255};
	vec2 S = (vec2)rwin.getSize();
	if(size < 2) return;

	bool type = extrs.type == Extremes::MAXFIRST;
	for(s32 i = 0, ec = extrs.ids.size(); i < ec; i++, type = !type) {
		std::vector<sf::Vertex> line = {};
		s32 j0 = extrs[i];
		s32 j2 = extrs[i+1];
		auto col = type ? red : blue;
		for(s32 j = extrs[i]; j != j2 ; j = (j+1)%size) {
			line.push_back({(*this)[j]*S, col});
		}
		line.push_back({(*this)[j2]*S, col});
		rwin.draw(&(line[0]), mmod(j2-j0, size)+1, sf::LineStrip);
	}
}*/


float Poly::Area() const {
	float s = 0;
	for(u32 i = 0; i < size; i++) {
		s += cross((*this)[i+1], (*this)[i]);
	}
	return s / 2.f;
}
vec2 Poly::AreaXCenter() const {
	vec2 s = {0.f, 0.f};
	vec2 a, b;
	for(u32 i = 0; i < size; i++) {
		a = (*this)[i], b = (*this)[i+1];
		s += cross(b, a) * (a + b);
	}
	return s / 6.f;
}

vec2 Poly::MassCenter() const {
	return AreaXCenter() / Area();
}

s32 MonotonicZones::InspectZone(Zone z, const Poly& poly, vec2 p) {
	float py = p.y;
	float ay = poly[z.a].y, by = poly[z.b].y;
	float mi = fmin(ay, by), ma = fmax(ay, by);

	if(z.type == SegType::ANY) return 0;
	bool u_d = z.type == SegType::UP;
	u32 c = (z.a+z.b)/2;
	if(py <= mi || py >= ma) return 0;
	float cy = poly[c].y;

	while(z.a != c) {
		if((p.y < cy)^u_d) {
			z.a = c;
		} else {
			z.b = c;
		}
		c = (z.a+z.b)/2;
		cy = poly[c].y;
	}
	vec2 A = poly[z.a], B = poly[z.b];
	return (cross(p-A,B-A) < 0) - u_d;
}

MonotonicZones Poly::DivideToMonotonics() const {
	if(!size) return {};

	std::vector<MonotonicZones::Zone> parts;
	SegType type0 = GetSegType(0), typeTemp = type0;
	s32 i0 = 0;
	while((typeTemp == SegType::ANY || typeTemp == type0) && i0 < size) typeTemp = GetSegType(++i0);
	if(i0 == size) return {{{0, (s32)size, type0}}};

	for(s32 i = i0, is = i0; i < i0 + size; i++) {
		SegType typeNext = GetSegType(i + 1);
		if(typeNext == SegType::ANY) typeNext = typeTemp;
		if(typeTemp != typeNext) {
			parts.push_back({is, i + 1, typeTemp});
			//printf("[%d %d] : %s\n", is, i + 1, typeTemp == SegType::UP ? "up" : typeTemp == SegType::DOWN ? "down" : "???");
			typeTemp = typeNext, is = i + 1;
		}
	}
	return {parts};
}

s32 Poly::IsInsideInt(const MonotonicZones& mz, vec2 p) const {
	s32 s = 0;
	for(auto z : mz.parts) {
		s32 ds = MonotonicZones::InspectZone(z, *this, p);
		//printf("[%d %d] -> %d\n", z.a, z.b, ds);
		s += ds;
	}
	return s;
}