#include "polygon.h"
#include "primitives.h"
#include <algorithm>
#include <numeric>

reindexed_cloud::reindexed_cloud(std::vector<u32> ids, const point_cloud* source) : std::vector<u32>(ids), source(source) { }
vec2 reindexed_cloud::satat(u32 i) const { return sat(at(i)); }
vec2 reindexed_cloud::sat(u32 i) const { return source->at(i); }


reindexed_cloud geom::to_sorted(const point_cloud& cloud) {
	u32 n = cloud.size();
	const point_cloud* t = &cloud;
	reindexed_cloud rc(std::vector<u32>(n), t);
	for(u32 i = 0; i < n; i++) rc[i] = i;
	std::sort(rc.begin(), rc.end(), [t](u32 a, u32 b) { return t->at(a).x < t->at(b).x || (t->at(a).x == t->at(b).x && t->at(a).y < t->at(b).y); });	
	return rc;
}
reindexed_cloud geom::minimal_hull(const reindexed_cloud& cloud) {
	u32 n = cloud.size();
	if(!n) return {};
	std::vector<u32> upper = {0}, lower = {0};
	for(u32 i = 1; i < n; i++) {
		u32 nu = upper.size(), nl = lower.size();
		vec2 p = cloud.satat(i), ulast = cloud.satat(upper[nu - 1]), llast = cloud.satat(lower[nl - 1]);

		s32 j = nu;
		for(; --j > 0;) {
			vec2 a = cloud.satat(upper[j - 1]), b = cloud.satat(upper[j]);
			float cr = cross(b - a, p - b);
			float d1 = dot(b - a, p - b);
			float d2 = dot(b - a, b - a);
			if(d2 < 1.e-7) printf("!");
			if(cr < 0 || (cr == 0 && d1 <= 0)) break;
		}
		if(cloud.sat(upper[0]) == cloud.sat(i))printf("!");
		upper.resize(j + 2);
		upper[j + 1] = i;

		j = nl;
		for(; --j > 0;) {
			vec2 a = cloud.satat(lower[j - 1]), b = cloud.satat(lower[j]);
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
	for(u32& i : upper) i = cloud.at(i);
	return {upper, cloud.source};
}
reindexed_cloud geom::minimal_hull(const point_cloud& cloud) {
	return minimal_hull(to_sorted(cloud));
}
poly reindexed_cloud::make_poly() const {
	std::vector<vec2> vecs;
	for(u32 i : *this) {
		vecs.push_back(source->at(i));
	}
	return {vecs};
}
std::pair<bool, poly> geom::verify_minimal_hull(const reindexed_cloud& cloud) {
	bool test = true;
	for(u32 n = cloud.size(), i = 0; i < n; i++) {
		vec2 a = cloud.satat(i), b = cloud.satat((i+1)%n), c = cloud.satat((i+2)%n);
		float cr = cross(b - a, c - b);
		float d = dot(b - a, c - b);
		if(cr > 0 || (cr == 0 && d < 0)) {
			printf("not convex : %u %u %u, cr: %g, d: %g\n", i, (i+1)%n, (i+2)%n, cr, d);
			//test = false;
		}
	}
	poly p = cloud.make_poly();
	auto mz = geom::divide_to_monotonics(p, {0, 1});
	for(u32 n = cloud.source->size(), i = 0; i < n; i++) {
		auto f = std::find(cloud.begin(), cloud.end(), i); 
		if(f != cloud.end()) continue;
		if(vec2 v = cloud.sat(i); geom::is_inside_val(p, mz, v, {0, 1}) <= 0) {
			printf("point outside: %u (%f, %f)\n", i, v.x, v.y);
			//test = flase;
		}
	}
	return {test, p};
}
std::tuple<bool, point_cloud, poly> geom::minimal_hull_test(u32 N) {
	point_cloud cloud;
	cloud.resize(N);
	for(u32 i = 0; i < N; i++) {
		vec2 rv = rand_vec2();
		float r = pow(log(1. - rv.x * .9f), 2.) * .08f, phi = rv.y * 2 * M_PI;
		cloud[i] = vec2(.5f + r * sin(phi), .5f + r * cos(phi));
		// vec2 rv = {(rand()%20)/20.f, (rand()%20)/20.f};
		// cloud[i] = rv * vec2(.8f, .8f) + vec2(.1f, .1f);
	}
	auto hull = hull_by_circular(to_circular_sorted(cloud));
	auto [r, poly] = verify_minimal_hull(hull);
	return {r, cloud, poly};
}
reindexed_cloud geom::to_circular_sorted(const point_cloud& cloud) {
 	u32 n = cloud.size();
 	reindexed_cloud rc(std::vector<u32>(n), &cloud);
 	for(u32 i = 0; i < n; i++) rc[i] = i;
 	if(n < 3) return rc;

	vec2 O = cloud.at(rc[0]);
	u32 im = 0;
	for (u32 i = 1; i < n; i++) {
		vec2 v = cloud.at(rc[i]);
		if(v.y == O.y ? v.x > O.x : v.y < O.y) {
			im = i, O = v;
		}
	}
	std::swap(rc[0], rc[im]);
	std::sort(rc.begin() + 1, rc.end(), [&cloud, O](u32 a, u32 b) {
		vec2 A = cloud.at(a) - O, B = cloud.at(b) - O;
		float phiA = atan2(A.x, A.y), phiB = atan2(B.x, B.y);
		float rA = len2(A), rB = len2(B);
		return rA == 0 ? rB > 0 :(rB != 0) && ((phiA != phiB) ? phiA < phiB : rA < rB);
	});

	return rc;
}
reindexed_cloud geom::hull_by_circular(const reindexed_cloud& cloud) {
	u32 n = cloud.size();
	if(n < 3) return cloud;
	reindexed_cloud ans = {{cloud.at(0)}, cloud.source};
	auto prev = [&](u32 i) {
		vec2 v = ans.satat(i);
		s32 j = i;
		while(--j, j && ans.satat(j) == v);
		return j;
	};
	for(s32 i = 1; i < n; ++i) {
		vec2 I = cloud.satat(i), J, K;
		u32 j = ans.size() - 1, k;
		for(;j > 0; j = k) {
			k = prev(j);
			if(k < 0) break;
		 	J = ans.satat(j), K = ans.satat(k);
		 	if(check_angle(K, J, I, 0, 1, 0, 0, 1)) break;
		}
		ans.resize(j + 2);
		ans[j + 1] = cloud.at(i);
	}
	return ans;
}

circle geom::welzl_2(vec2 a, vec2 b) {
	return {a, b};
}
circle geom::welzl_3(vec2 a, vec2 b, vec2 c) {
	circle ab = welzl_2(a, b), bc = welzl_2(b, c), ca = welzl_2(c, a);
	if(ab.inside(c)) return ab;
	if(bc.inside(a)) return bc;
	if(ca.inside(b)) return ca;

	vec2 o = circumcenter(a, b, c);
	return {a, 2 * o - a};
}
circle geom::welzl_trivial(const reindexed_cloud& rng) {
	u32 s = rng.size();
	if(s == 0) {
		return {};
	} else if(vec2 a = rng.satat(0); s == 1) {
		return {a, a};
	} else if(vec2 b = rng.satat(1); s == 2) {
		return welzl_2(a, b);
	} else {
		vec2 c = rng.satat(2);
		return welzl_3(a, b, c);
	}
}
circle geom::welzl(reindexed_cloud P, reindexed_cloud R) {
	u32 ps = P.size();
	if(ps == 0 || R.size() == 3) return welzl_trivial(R);
	s32 p = P.at(ps-1); P.pop_back();
	circle D = welzl(P, R);
	if(!D.inside(P.sat(p))) {
		R.push_back(p);
		D = welzl(P, R);
	}
	return D;
}
circle geom::welzl(point_cloud cloud) {
	std::sort(cloud.begin(), cloud.end(), [](vec2 a, vec2 b){return a.x == b.x ? a.x < b.x : a.y < b.y; });
	cloud.erase(std::unique(cloud.begin(), cloud.end()), cloud.end());

	std::random_shuffle(cloud.begin(), cloud.end());
	std::vector<u32> ids(cloud.size());	
	std::iota(ids.begin(), ids.end(), 0), std::random_shuffle(ids.begin(), ids.end());
	reindexed_cloud P = {ids, &cloud}, R = {{}, &cloud};

	circle C = welzl(P, R);
	// u32 q = 0;
	// for(auto p : cloud) {
	// 	if(!C.inside(p)) {
	// 		q++;
	// 	}
	// }
	// if(q) printf("%u / %lu\n", q, cloud.size());

	return C;
}