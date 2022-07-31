#include "utils.h"
#include "primitives.h"

float randf() { return (float)rand() / RAND_MAX; }
vec3 rand_unit() {
	float y = randf()*2-1;
	float phi = randf()*M_PI*2;
	float r = sqrt(1-y*y);
	return {r*sinf(phi), y, r*cosf(phi)};
}
vec2 rand_vec2() { return {randf(), randf()}; }

vec3 operator*(vec3 a, vec3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
vec3 operator*(vec3 a, float b) { return {a.x * b, a.y * b, a.z * b}; }
vec3 operator*(float a, vec3 b) { return {a * b.x, a * b.y, a * b.z}; }
vec3 operator/(vec3 a, vec3 b) { return {a.x / b.x, a.y / b.x, a.z / b.z}; }
vec3 operator/(vec3 a, float b) { return {a.x / b, a.y / b, a.z / b}; }
vec3 operator/(float a, vec3 b) { return {a / b.x, a / b.y, a / b.z}; }

vec2 operator*(vec2 a, vec2 b) { return {a.x * b.x, a.y * b.y}; }
vec2 operator*(vec2 a, float b) { return {a.x * b, a.y * b}; }
vec2 operator*(float a, vec2 b) { return {a * b.x, a * b.y}; }
vec2 operator/(vec2 a, vec2 b) { return {a.x / b.x, a.y / b.y}; }
vec2 operator/(vec2 a, float b) { return {a.x / b, a.y / b}; }
vec2 operator/(float a, vec2 b) { return {a / b.x, a / b.y}; }


float dot(vec2 a, vec2 b) { return a.x * b.x + a.y * b.y; }
float dot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
vec3 normalize(vec3 v, vec3 d, float eps) {
	float l = len(v);
	return l < eps ? d : v / l;
}
vec2 normalize(vec2 v, vec2 d, float eps) {
	float l = len(v);
	return l < eps ? d : v / l;
}
vec3 utof(vec3u v) { return vec3(v.x, v.y, v.z); }
vec3 cross(vec3 a, vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
float cross(vec2 a, vec2 b) { return a.x * b.y - a.y * b.x; }

float len2(vec2 v) { return dot(v, v); }
float len2(vec3 v) { return dot(v, v); }
float len(vec2 v) { return sqrt(len2(v)); }
float len(vec3 v) { return sqrt(len2(v)); }

s32 mmod(s32 a, s32 n) {
	return (a%n+n)%n;
}

bool check_angle(vec2 A, vec2 B, vec2 C, bool l, bool r, bool f, bool b, bool o) {
	float cr = cross(B - A, C - B);
	float d = dot(B - A, C - A);
	return cr > 0 ? l : cr < 0 ? r : d > 0 ? f : d < 0 ? b : o;
}

vec2 circumcenter(vec2 A, vec2 B, vec2 C) {
	float k1 = (len2(B) - len2(A)) / 2;
	float k2 = (len2(C) - len2(A)) / 2;
	vec2 v1 = B - A, v2 = C - A;
	auto mat = mat2x2::from_rows(v1, v2);
	//if(fabs(mat.det()) <= EPS) return A;
	return mat.inv() * vec2(k1, k2);
}

float lerp(float a, float b, float t) { return a + (b - a) * t; }
vec2 lerp(vec2 a, vec2 b, float t) { return a + (b - a) * t; }
vec3 lerp(vec3 a, vec3 b, float t) { return a + (b - a) * t; }

vec2 lrot(vec2 a) { return {-a.y, a.x}; }
vec2 rrot(vec2 a) { return {a.y, -a.x}; }
float rlerp(float a, float b, float c) { return a == b ? .5f : (c - a) / (b - a); }

s32 sign(float v) {
	return (v > 0) - (v < 0);
}

s32 rotsign(vec2 a, vec2 b, vec2 c) {
	return sign(cross(b - a, c - b));
}

cross_type check_intersection(vec2 a, vec2 b, vec2 c, vec2 d) {
	if(a == c || b == c || a == d || b == d) return cross_type::BROKEN;

	float ab2 = dot(b - a, b - a);
	float abac = dot(b - a, c - a);
	float abad = dot(b - a, d - a);
	bool cl_c = fabs(cross(b - a, c - a)) < EPS;
	bool cl_d = fabs(cross(b - a, d - a)) < EPS;

	if(cl_c && cl_d) {
		float mi, ma;
		if(abac < abad) {
			mi = abac, ma = abad;
		} else {
			mi = abad, ma = abac;
		}
		return mi < ab2 + EPS && ma > -EPS ? cross_type::BROKEN : cross_type::NONE;
	} else if(cl_c) {
		if(abac > -EPS && abac < ab2 + EPS) return cross_type::BROKEN;
	} else if(cl_d) {
		if(abad > -EPS && abad < ab2 + EPS) return cross_type::BROKEN;
	}

	s32 c0 = rotsign(a, d, b), c1 = rotsign(d, b, c), c2 = rotsign(b, c, a), c3 = rotsign(c, a, d);
	if (c0 == c1 && c1 == c2 && c2 == c3) {
		return cross_type::HAS;
	} else {
		return cross_type::NONE;
	}
}
