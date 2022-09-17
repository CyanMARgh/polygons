#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>
#include "geometry.h"
#include <algorithm>
#include <vector>
#include <numeric>

const float EPS = 1e-7;

float randf();
vec3 rand_unit();
vec2 rand_unit2();
vec2 rand_vec2();

//vec3
vec3 operator*(vec3 A, vec3 B);
vec3 operator*(vec3 A, float b);
vec3 operator*(float a, vec3 B);
vec3 operator/(vec3 A, vec3 B);
vec3 operator/(vec3 A, float b);
vec3 operator/(float a, vec3 B);

//vec2
vec2 operator*(vec2 A, vec2 B);
vec2 operator*(vec2 A, float b);
vec2 operator*(float a, vec2 B);
vec2 operator/(vec2 A, vec2 B);
vec2 operator/(vec2 A, float b);
vec2 operator/(float a, vec2 B);

float dot(vec2 a, vec2 b);
float dot(vec3 a, vec3 b);
vec3 normalize(vec3 v, vec3 d = vec3(1,0,0), float eps = 1e-9);
vec2 normalize(vec2 v, vec2 d = vec2(1,0), float eps = 1e-9);
vec3 utof(vec3u v);

float len2(vec2 v);
float len2(vec3 v);

float len(vec2 v);
float len(vec3 v);

vec3 cross(vec3 a, vec3 b);
float cross(vec2 a, vec2 b);

s32 mmod(s32 a, s32 n);

bool check_angle(vec2 A, vec2 B, vec2 C, bool l, bool r, bool f, bool b, bool o);
vec2 circumcenter(vec2 A, vec2 B, vec2 C);

float lerp(float a, float b, float t);
vec2 lerp(vec2 a, vec2 b, float t);
vec3 lerp(vec3 a, vec3 b, float t);

vec2 lrot(vec2 a);
vec2 rrot(vec2 a);
float rlerp(float a, float b, float c);

enum class cross_type { 
	NONE = 0,
	HAS = 1,
	BROKEN = 2
};

s32 sign(float v);
cross_type check_intersection(vec2 a, vec2 b, vec2 c, vec2 d);
vec2 find_intersection(vec2 a, vec2 b, vec2 c, vec2 d);

template<typename T>
std::pair<std::vector<u32>, std::vector<u32>> make_permutation(const std::vector<T>& P, std::function<bool(T, T)> cmp) {
	u32 n = P.size();
	std::vector<u32> ids(n), rids(n);
	std::iota(ids.begin(), ids.end(), 0);
	std::sort(ids.begin(), ids.end(), [&P, &cmp] (u32 a, u32 b) {return cmp(P[a], P[b]); });
	for(u32 i = 0; i < n; i++) rids[ids[i]] = i;
	return {ids, rids};
}