#include "polygon.h"
#include "box2.h"
#include <algorithm>

//base
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

// mass center & area
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

// "is inside?"
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

//convex hull
ReindexedCloud::ReindexedCloud(std::vector<u32> ids, const PointCloud* source) : std::vector<u32>(ids), source(source) { }

ReindexedCloud PointCloud::ToSorted() const {
	u32 n = size();
	auto t = this;
	ReindexedCloud rc(std::vector<u32>(n), t);
	for(u32 i = 0; i < n; i++) rc[i] = i;
	std::sort(rc.begin(), rc.end(), [t](u32 a, u32 b) { return t->at(a).x < t->at(b).x || (t->at(a).x == t->at(b).x && t->at(a).y < t->at(b).y); });	
	return rc;
}
// fix to nlogn
ReindexedCloud ReindexedCloud::MinimalHull() const {
	u32 n = size();
	if(!n) return {};
	std::vector<u32> upper = {0}, lower = {0};
	for(u32 i = 1; i < n; i++) {
		u32 nu = upper.size(), nl = lower.size();
		vec2 p = source->at(at(i)), ulast = source->at(at(upper[nu - 1])), llast = source->at(at(lower[nl - 1]));

		s32 j = nu;
		for(; --j > 0;) {
			vec2 a = source->at(at(upper[j - 1])), b = source->at(at(upper[j]));
			float cr = cross(b - a, p - b);
			float d1 = dot(b - a, p - b);
			float d2 = dot(b - a, b - a);
			if(d2 < 1.e-7) printf("!");
			if(cr < 0 || (cr == 0 && d1 <= 0)) break;
		}
		if(source->at(upper[0]) == source->at(i))printf("!");
		upper.resize(j + 2);
		upper[j + 1] = i;

		j = nl;
		for(; --j > 0;) {
			vec2 a = source->at(at(lower[j - 1])), b = source->at(at(lower[j]));
			float cr = cross(b - a, p - b);
			float d1 = dot(b - a, p - b);
			float d2 = dot(b - a, b - a);
			if(d2 == 0) printf("!");
			if(cr > 0 || (cr == 0 && d1 <= 0)) break;
		}
		lower.resize(j + 2);
		lower[j + 1] = i;
	}
	for(s32 nl = lower.size(), i = nl - 2; i > 0; --i) {
		upper.push_back(lower[i]);
	}
	for(u32& i : upper) i = at(i);
	return {upper, source};
}
ReindexedCloud PointCloud::MinimalHull() const {
	return ToSorted().MinimalHull();
}
Poly ReindexedCloud::MakePoly() const {
	std::vector<vec2> vecs;
	for(u32 i : *this) {
		vecs.push_back(source->at(i));
	}
	return {vecs};
}
std::pair<bool, Poly> ReindexedCloud::VerifyMinimalHull() const {
	bool test = true;
	for(u32 n = size(), i = 0; i < n; i++) {
		vec2 a = (*source)[(*this)[i]], b = (*source)[(*this)[(i+1)%n]], c = (*source)[(*this)[(i+2)%n]];
		float cr = cross(b - a, c - b);
		float d = dot(b - a, c - b);
		if(cr > 0 || (cr == 0 && d < 0)) {
			printf("not convex : %u %u %u, cr: %g, d: %g\n", i, (i+1)%n, (i+2)%n, cr, d);
			//test = false;
		}
	}
	Poly p = MakePoly();
	auto mz = p.DivideToMonotonics();
	for(u32 n = source->size(), i = 0; i < n; i++) {
		auto f = std::find(begin(), end(), i); 
		if(f != end()) continue;
		if(vec2 v = source->at(i); p.IsInsideInt(mz, v) <= 0) {
			printf("point outside: %u (%f, %f)\n", i, v.x, v.y);
			//test = flase;
		}
	}
	return {test, p};
}
std::tuple<bool, PointCloud, Poly> ReindexedCloud::MinimalHullTest(u32 N) {
	PointCloud cloud;
	cloud.resize(N);
	for(u32 i = 0; i < N; i++) {
		vec2 rv = RandVec2();
		float r = pow(log(1. - rv.x * .9f), 2.) * .08f, phi = rv.y * 2 * M_PI;
		cloud[i] = vec2(.5f + r * sin(phi), .5f + r * cos(phi));
		// vec2 rv = {(rand()%20)/20.f, (rand()%20)/20.f};
		// cloud[i] = rv * vec2(.8f, .8f) + vec2(.1f, .1f);
	}
	auto hull = cloud.ToCircularSorted().HullByCircular();
	auto [r, poly] = hull.VerifyMinimalHull();
	return {r, cloud, poly};
}

ReindexedCloud PointCloud::ToCircularSorted() const {
 	u32 n = size();
 	ReindexedCloud rc(std::vector<u32>(n), this);
 	for(u32 i = 0; i < n; i++) rc[i] = i;
 	if(n < 3) return rc;

	vec2 O = at(rc[0]);
	u32 im = 0;
	for (u32 i = 1; i < n; i++) {
		vec2 v = at(rc[i]);
		if(v.y == O.y ? v.x > O.x : v.y < O.y) {
			im = i, O = v;
		}
	}
	std::swap(rc[0], rc[im]);
	std::sort(rc.begin() + 1, rc.end(), [this, O](u32 a, u32 b) {
		vec2 A = at(a) - O, B = at(b) - O;
		float phiA = atan2(A.x, A.y), phiB = atan2(B.x, B.y);
		float rA = len2(A), rB = len2(B);
		return rA == 0 ? rB > 0 :(rB != 0) && ((phiA != phiB) ? phiA < phiB : rA < rB);
	});

	return rc;
}
ReindexedCloud ReindexedCloud::HullByCircular() const {
	u32 n = size();
	if(n < 3) return *this;
	std::vector<u32> ans = {at(0)};
	auto prev = [this, &ans](u32 i) {
		vec2 v = source->at(ans.at(i));
		s32 j = i;
		while(--j, j && source->at(ans.at(j)) == v);
		return j;
	};
	for(s32 i = 1; i < n; ++i) {
		vec2 I = source->at(at(i)), J, K;
		u32 j = ans.size() - 1, k;
		for(;j > 0; j = k) {
			k = prev(j);
			if(k < 0) break;
		 	J = source->at(ans[j]), K = source->at(ans[k]);
		 	if(CheckAngle(K, J, I, 0, 1, 0, 0, 1)) break;
		}
		ans.resize(j + 2);
		ans[j + 1] = at(i);
	}
	return {ans, source};
}



