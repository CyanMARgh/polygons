#include "polygon.h"
#include <algorithm>
#include <numeric>

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

s32 cloud_range::size() const {
	return b - a;
}
vec2 cloud_range::at(u32 i) const {
	return source->at(a + i);
}
bool circle::inside(vec2 p) const {
	return len2(p - f1) + len2(p - f2) <= len2(f2 - f1);
}
float circle::rad() const {
	return len(f2 - f1) * .5f;
}
vec2 circle::center() const {
	return (f1 + f2) * .5f;
}

void circle::draw(sf::RenderWindow& rwin, sf::CircleShape& spr, box2 box) const {
	spr.setPosition(box * ((f2 + f1) * .5f));
	float R = len(f2 - f1) * box.s.x * .5f;
	spr.setRadius(R);
	spr.setOrigin(R, R);
	rwin.draw(spr);
}
circle welzl::trivial2(vec2 a, vec2 b) {
	return {a, b};
}
circle welzl::trivial3(vec2 a, vec2 b, vec2 c) {
	circle ab = trivial2(a, b), bc = trivial2(b, c), ca = trivial2(c, a);
	if(ab.inside(c)) return ab;
	if(bc.inside(a)) return bc;
	if(ca.inside(b)) return ca;

	vec2 o = circumcenter(a, b, c);
	return {a, 2 * o - a};
}
circle welzl::trivial(const reindexed_cloud& rng) {
	u32 s = rng.size();
	if(s == 0) {
		return {};
	} else if(vec2 a = rng.satat(0); s == 1) {
		return {a, a};
	} else if(vec2 b = rng.satat(1); s == 2) {
		return trivial2(a, b);
	} else {
		vec2 c = rng.satat(2);
		return trivial3(a, b, c);
	}
}
circle welzl::get(reindexed_cloud P, reindexed_cloud R) {
	u32 ps = P.size();
	if(ps == 0 || R.size() == 3) return trivial(R);
	s32 p = P.at(ps-1); P.pop_back();
	circle D = get(P, R);
	if(!D.inside(P.sat(p))) {
		R.push_back(p);
		D = get(P, R);
	}
	return D;
}
circle welzl::get(point_cloud cloud) {
	std::sort(cloud.begin(), cloud.end(), [](vec2 a, vec2 b){return a.x == b.x ? a.x < b.x : a.y < b.y; });
	cloud.erase(std::unique(cloud.begin(), cloud.end()), cloud.end());

	std::random_shuffle(cloud.begin(), cloud.end());
	std::vector<u32> ids(cloud.size());	
	std::iota(ids.begin(), ids.end(), 0), std::random_shuffle(ids.begin(), ids.end());
	reindexed_cloud P = {ids, &cloud}, R = {{}, &cloud};

	circle C = get(P, R);
	// u32 q = 0;
	// for(auto p : cloud) {
	// 	if(!C.inside(p)) {
	// 		q++;
	// 	}
	// }
	// if(q) printf("%u / %lu\n", q, cloud.size());

	return C;
}