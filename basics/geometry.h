#pragma once

#include <SFML/System.hpp>

//basic types
typedef uint8_t u8;
typedef uint32_t u32;
typedef int32_t s32;

typedef sf::Vector2u vec2u;
typedef sf::Vector2f vec2;
typedef sf::Vector3<u32> vec3u;
typedef sf::Vector3f vec3;

//primitives
struct line;
struct circle;
struct box2;
struct mat2x2;

//complex structures
struct poly;
//struct poly_group;
struct point_cloud;
struct monotonic_zones;
struct reindexed_cloud;
struct intersection;
struct intersection_list;
enum class seg_type;
//struct graph;
