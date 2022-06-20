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

void Poly::DrawPoly(sf::RenderWindow& rwin, Box2 box) const {
	vec2 S = (vec2)rwin.getSize();
	if(size < 1) return;
	sf::Vertex *vlines = new sf::Vertex[size + 1];
	for(u32 i = 0; i <= size; i++) {
		vlines[i] = box * (*this)[i];
	}

	rwin.draw(vlines, size + 1, sf::LineStrip);
	delete[] vlines;
}

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
	float py = p.y, ay = poly[z.a].y, by = poly[z.b].y, cy;
	float mi = fmin(ay, by), ma = fmax(ay, by);

	if(py <= mi || py >= ma || z.type == SegType::ANY) return 0;
	bool u_d = z.type == SegType::UP;

	for(u32 c;;) {
		c = (z.a + z.b) / 2;
		cy = poly[c].y;
		if(z.a == c) break;
		((p.y < cy) ^ u_d ? z.a : z.b) = c;
	}

	vec2 A = poly[z.a], B = poly[z.b];
	return u_d - (cross(p-A,B-A) < 0);
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
			typeTemp = typeNext, is = i + 1;
		}
	}
	return {parts};
}

s32 Poly::IsInsideInt(const MonotonicZones& mz, vec2 p) const {
	s32 s = 0;
	for(auto z : mz.parts) {
		s32 ds = MonotonicZones::InspectZone(z, *this, p);
		s += ds;
	}
	return s;
}

SegType Poly::GetSegType(s32 i) const {
	float y0 = (*this)[i].y, y1 = (*this)[i + 1].y;
	return y0 < y1 ? SegType::UP : y0 > y1 ? SegType::DOWN : SegType::ANY; 
}

void MonotonicZones::print() {
	for(auto z : parts) {
		printf("[%d,%d] ", z.a, z.b);
	}
	printf("\n");
}
IndexedCloud ToSorted(const PointCloud& cloud) {
	u32 n = cloud.size();
	IndexedCloud icloud = IndexedCloud(n);
	for(u32 i = 0; i < n; i++) { icloud[i] = {cloud[i], i}; }
	std::sort(icloud.begin(), icloud.end(), [](ipoint a, ipoint b) { return a.p.x < b.p.x || (a.p.x == b.p.x && a.p.y < b.p.y); });	
	return icloud;
}

std::vector<u32> MinimalHull(const IndexedCloud& cloud) {
	u32 n = cloud.size();
	if(!n) return {};
	std::vector<u32> upper = {0}, lower = {0};
	for(u32 i = 1; i < n; i++) {
		u32 nu = upper.size(), nl = lower.size();
		vec2 p = cloud[i].p, ulast = cloud[upper[nu - 1]].p, llast = cloud[lower[nl - 1]].p;

		{
			s32 j = nu;
			for(;--j > 0;) {
				vec2 a = cloud[upper[j - 1]].p, b = cloud[upper[j]].p;
				float cr = cross(b - a, p - b);
				float d1 = dot(b - a, b - a);
				if(cr < 0 || (cr == 0 && d1 <= 0)) break;
			}
			upper.resize(j + 2);
			upper[j + 1] = i;
		}
		{
			s32 j = nl;
			for(;--j > 0;) {
				vec2 a = cloud[lower[j - 1]].p, b = cloud[lower[j]].p;
				float cr = cross(b - a, p - b);
				float d1 = dot(b - a, b - a);
				if(cr > 0 || (cr == 0 && d1 <= 0)) break;
			}
			lower.resize(j + 2);
			lower[j + 1] = i;
		}
	}
	for(s32 nl = lower.size(), i = nl - 2; i > 0; --i) {
		upper.push_back(lower[i]);
	}
	return upper;
}
std::vector<u32> MinimalHull(const PointCloud& cloud) {
	auto icloud = ToSorted(cloud);
	auto ids = MinimalHull(icloud);
	std::vector<u32> ans;
	for(u32 i = 0; i < ids.size(); i++) {
		ans.push_back(icloud[ids[i]].id);
	}
	return ans;
}

Poly MakePoly(const PointCloud& cloud, const std::vector<u32>& ids) {
	std::vector<vec2> vecs;
	for(u32 i : ids) {
		vecs.push_back(cloud[i]);
	}
	return {vecs};
}

Poly SortedPoly(const PointCloud& cloud) {
	auto icloud = ToSorted(cloud);
	std::vector<vec2> v;
	for(auto i : icloud) {
		v.push_back(i.p);
	}
	return {v};
}

bool VerifyMinimalHull(const std::vector<u32>& ids, const PointCloud& cloud) {
	for(u32 n = ids.size(), i = 0; i < n; i++) {
		vec2 a = cloud[ids[i]], b = cloud[ids[(i+1)%n]], c = cloud[ids[(i+2)%n]];
		float cr = cross(b - a, c - b);
		float d = dot(b - a, c - b);
		if(cr > 0 || (cr == 0 && d > 0)) {
			printf("not convex : %u %u %u\n", i, (i+1)%n, (i+2)%n);
			return false;
		}
	}
	Poly p = MakePoly(cloud, ids);
	auto mz = p.DivideToMonotonics();
	for(u32 n = cloud.size(), i = 0; i < n; i++) {
		auto f = std::find(ids.begin(), ids.end(), i); 
		if(f != ids.end()) continue;
		if(p.IsInsideInt(mz, cloud[i]) > 0) {
			printf("point outside: %u\n", i);
			return false;
		}
	}
	return true;
}