#include "polygon.h"
#include <algorithm>
#include <numeric>

//base
poly::poly() {
	points = {}, size = 0;
}
poly::poly(std::vector<vec2> other) : points(std::move(other)) { 
	size = points.size();
}
poly& poly::operator=(std::vector<vec2> other) {
	points = std::move(other);
	size = other.size();
	return *this;
}
poly::poly(const poly& P) {
	points = P.points;
	size = P.size;
}
void poly::add(vec2 p) {
	points.push_back(p);
	++size;
}
vec2& poly::operator[](s32 i) {
	return points[mmod(i, points.size())];
}
vec2 poly::operator[](s32 i) const {
	return points[mmod(i, points.size())];
}
void poly::draw(sf::RenderWindow& rwin, box2 box) const {
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
float poly::area() const {
	float s = 0;
	for(u32 i = 0; i < size; i++) {
		s += cross((*this)[i+1], (*this)[i]);
	}
	return s / 2.f;
}
vec2 poly::area_X_center() const {
	vec2 s = {0.f, 0.f};
	vec2 a, b;
	for(u32 i = 0; i < size; i++) {
		a = (*this)[i], b = (*this)[i+1];
		s += cross(b, a) * (a + b);
	}
	return s / 6.f;
}
vec2 poly::mass_center() const {
	return area_X_center() / area();
}

// "is inside?"
// s32 monotonic_zones::inspect_zone(zone z, const poly& P, vec2 p) {
// 	float py = p.y, ay = P[z.a].y, by = P[z.b].y, cy;
// 	float mi = fmin(ay, by), ma = fmax(ay, by);

// 	if(py <= mi || py >= ma || z.type == seg_type::ANY) return 0;
// 	bool u_d = z.type == seg_type::UP;

// 	for(u32 c;;) {
// 		c = (z.a + z.b) / 2;
// 		cy = P[c].y;
// 		if(z.a == c) break;
// 		((p.y < cy) ^ u_d ? z.a : z.b) = c;
// 	}

// 	vec2 A = P[z.a], B = P[z.b];
// 	return u_d - (cross(p-A,B-A) < 0);
// }
s32 monotonic_zones::inspect_zone(zone z, const poly& P, vec2 p, vec2 n) {
	float py = dot(p, n), ay = dot(P[z.a], n), by = dot(P[z.b], n), cy;
	float mi = fmin(ay, by), ma = fmax(ay, by);

	if(py <= mi || py >= ma || z.type == seg_type::ANY) return 0;
	bool u_d = z.type == seg_type::UP;

	for(u32 c;;) {
		c = (z.a + z.b) / 2;
		cy = dot(P[c], n);
		if(z.a == c) break;
		((dot(p, n) < cy) ^ u_d ? z.a : z.b) = c;
	}

	vec2 A = P[z.a], B = P[z.b];
	float t1 = rlerp(dot(A, n), dot(B, n), dot(p, n));
	float t2 = lerp(cross(A, n), cross(B, n), t1) - cross(p, n);

	intersection I = {t1, t2, z.a};	
	return u_d - ((t2 < 0) ^ u_d);
}
monotonic_zones poly::divide_to_monotonics(vec2 n) const {
	if(!size) return {};

	std::vector<monotonic_zones::zone> parts;
	s32 i0 = 0;

	seg_type type0, type_temp;
	while((type0 = get_seg_type(i0, n)) == seg_type::ANY && i0 < size) i0++;
	if(i0 == size) return {{{0, (s32)size, type0}}};
	i0++;
	while(type_temp = get_seg_type(i0, n), (type_temp == seg_type::ANY || type_temp == type0) && i0 <= size) i0++;

	for(s32 i = i0, is = i0; i < i0 + size; i++) {
		seg_type type_next = get_seg_type(i + 1, n);
		if(type_next == seg_type::ANY) continue;
		if(type_temp != type_next) {
			parts.push_back({is, i + 1, type_temp});
			type_temp = type_next, is = i + 1;
		}
	}
	return {parts};
}
s32 poly::is_inside_val(const monotonic_zones& mz, vec2 p) const {
	s32 s = 0;
	for(auto z : mz.parts) {
		s32 ds = monotonic_zones::inspect_zone(z, *this, p);
		s += ds;
	}
	return s;
}
seg_type poly::get_seg_type(s32 i, vec2 n) const {
	float q = dot(n, (*this)[i+1]-(*this)[i]);
	return q > 0 ? seg_type::UP : q < 0 ? seg_type::DOWN : seg_type::ANY; 
}
void monotonic_zones::print() {
	for(auto z : parts) {
		printf("[%d,%d] ", z.a, z.b);
	}
	printf("\n");
}

//convex hull
reindexed_cloud::reindexed_cloud(std::vector<u32> ids, const point_cloud* source) : std::vector<u32>(ids), source(source) { }
vec2 reindexed_cloud::satat(u32 i) const { return sat(at(i)); }
vec2 reindexed_cloud::sat(u32 i) const { return source->at(i); }

reindexed_cloud point_cloud::to_sorted() const {
	u32 n = size();
	auto t = this;
	reindexed_cloud rc(std::vector<u32>(n), t);
	for(u32 i = 0; i < n; i++) rc[i] = i;
	std::sort(rc.begin(), rc.end(), [t](u32 a, u32 b) { return t->at(a).x < t->at(b).x || (t->at(a).x == t->at(b).x && t->at(a).y < t->at(b).y); });	
	return rc;
}
// fix to nlogn
reindexed_cloud reindexed_cloud::minimal_hull() const {
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
reindexed_cloud point_cloud::minimal_hull() const {
	return to_sorted().minimal_hull();
}
poly reindexed_cloud::make_poly() const {
	std::vector<vec2> vecs;
	for(u32 i : *this) {
		vecs.push_back(source->at(i));
	}
	return {vecs};
}
std::pair<bool, poly> reindexed_cloud::verify_minimal_hull() const {
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
	poly p = make_poly();
	auto mz = p.divide_to_monotonics();
	for(u32 n = source->size(), i = 0; i < n; i++) {
		auto f = std::find(begin(), end(), i); 
		if(f != end()) continue;
		if(vec2 v = source->at(i); p.is_inside_val(mz, v) <= 0) {
			printf("point outside: %u (%f, %f)\n", i, v.x, v.y);
			//test = flase;
		}
	}
	return {test, p};
}
std::tuple<bool, point_cloud, poly> reindexed_cloud::minimal_hull_test(u32 N) {
	point_cloud cloud;
	cloud.resize(N);
	for(u32 i = 0; i < N; i++) {
		vec2 rv = rand_vec2();
		float r = pow(log(1. - rv.x * .9f), 2.) * .08f, phi = rv.y * 2 * M_PI;
		cloud[i] = vec2(.5f + r * sin(phi), .5f + r * cos(phi));
		// vec2 rv = {(rand()%20)/20.f, (rand()%20)/20.f};
		// cloud[i] = rv * vec2(.8f, .8f) + vec2(.1f, .1f);
	}
	auto hull = cloud.to_circular_sorted().hull_by_circular();
	auto [r, poly] = hull.verify_minimal_hull();
	return {r, cloud, poly};
}

reindexed_cloud point_cloud::to_circular_sorted() const {
 	u32 n = size();
 	reindexed_cloud rc(std::vector<u32>(n), this);
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
reindexed_cloud reindexed_cloud::hull_by_circular() const {
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
		 	if(check_angle(K, J, I, 0, 1, 0, 0, 1)) break;
		}
		ans.resize(j + 2);
		ans[j + 1] = at(i);
	}
	return {ans, source};
}

//minimum circle
s32 cloud_range::size() const {
	return b - a;
}
vec2 cloud_range::at(u32 i) const {
	return source->at(a + i);
}
bool circle::inside(vec2 p) const {
	return len2(p - c) <= r2;
}
void circle::draw(sf::RenderWindow& rwin, sf::CircleShape& spr, box2 box) const {
	spr.setPosition(box * c);
	float R = sqrt(r2) * box.s.x;
	spr.setRadius(R);
	spr.setOrigin(R, R);
	rwin.draw(spr);
}
vec2 trivial3(vec2 a, vec2 b, vec2 c) {
	// if(a == b) return (b + c) / 2;
	// if(b == c || a == c) return (a + b) / 2;
	if(dot(a, b) + 4 * len2(c) <= dot(a + b, c)) return (a + b) / 2;
	if(dot(a, c) + 4 * len2(b) <= dot(a + c, b)) return (a + c) / 2;
	if(dot(b, c) + 4 * len2(a) <= dot(b + c, a)) return (b + c) / 2;
	return circumcenter(a, b, c);
}
circle trivial(const reindexed_cloud& rng) {
	u32 s = rng.size();
	if(s <= 0) {
		return {};
	} else if(vec2 a = rng.satat(0); s == 1) {
		return {a};
	} else if(vec2 b = rng.satat(1); s == 2) {
		return {(a + b) / 2, len2((a - b) / 2)};
	} else {
		vec2 c = rng.satat(2);
		vec2 o = trivial3(a, b, c);
		return {o, len2(o - a)};
	}
}
circle welzl(reindexed_cloud P, reindexed_cloud R) {
	u32 ps = P.size();
	if(ps == 0 || R.size() == 3) return trivial(R);
	s32 p = P.at(ps-1); P.pop_back();
	circle D = welzl(P, R);
	if(!D.inside(P.sat(p))) {
		R.push_back(p);
		D = welzl(P, R);
	}
	return D;
}
circle welzl(point_cloud cloud) {
	std::sort(cloud.begin(), cloud.end(), [](vec2 a, vec2 b){return a.x == b.x ? a.x < b.x : a.y < b.y; });
	cloud.erase(std::unique(cloud.begin(), cloud.end()), cloud.end());

	std::random_shuffle(cloud.begin(), cloud.end());
	std::vector<u32> ids(cloud.size());	
	std::iota(ids.begin(), ids.end(), 0), std::random_shuffle(ids.begin(), ids.end());
	reindexed_cloud P = {ids, &cloud}, R = {{}, &cloud};
	return welzl(P, R);
}
