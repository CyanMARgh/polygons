#include "utils.h"

float Randf() { return (float)rand() / RAND_MAX; }
vec3 RandUnit() {
	float y = Randf()*2-1;
	float phi = Randf()*M_PI*2;
	float r = sqrt(1-y*y);
	return {r*sinf(phi), y, r*cosf(phi)};
}
vec2 RandVec2() {
	return {Randf(), Randf()};
}


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
	float l = sqrt(dot(v, v));
	return l < eps ? d : v / l;
}
vec3 utof(vec3u v) { return vec3(v.x, v.y, v.z); }
vec3 cross(vec3 a, vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
float cross(vec2 a, vec2 b) { return a.x * b.y - a.y * b.x; }

float len2(vec2 v) { return dot(v, v); }
float len2(vec3 v) { return dot(v, v); }

s32 mmod(s32 a, s32 n) {
	return (a%n+n)%n;
}

bool CheckAngle(vec2 A, vec2 B, vec2 C, bool l, bool r, bool f, bool b, bool o) {
	float cr = cross(B - A, C - B);
	float d = dot(B - A, C - A);
	return cr > 0 ? l : cr < 0 ? r : d > 0 ? f : d < 0 ? b : o;
}
