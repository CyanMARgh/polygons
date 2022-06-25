#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>

typedef uint8_t u8;
typedef uint32_t u32;
typedef int32_t s32;

typedef sf::Vector2u vec2u;
typedef sf::Vector2f vec2;
typedef sf::Vector3<u32> vec3u;
typedef sf::Vector3f vec3;

float Randf();
vec3 RandUnit();
vec2 RandVec2();

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
vec3 utof(vec3u v);

float len2(vec2 v);
float len2(vec3 v);


vec3 cross(vec3 a, vec3 b);
float cross(vec2 a, vec2 b);

s32 mmod(s32 a, s32 n);

bool CheckAngle(vec2 A, vec2 B, vec2 C, bool l, bool r, bool f, bool b, bool o);