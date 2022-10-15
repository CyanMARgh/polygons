  #pragma once

#include <SFML/System.hpp>

//basic types
typedef uint8_t u8;
typedef int8_t s8;
typedef uint32_t u32;
typedef int32_t s32;

typedef sf::Vector2i vec2s;
typedef sf::Vector2u vec2u;
typedef sf::Vector2f vec2;
typedef sf::Vector3<u32> vec3u;
typedef sf::Vector3f vec3;

//primitives
struct Line;
struct Circle;
struct Box2;
struct Mat2x2;

//complex structures
struct Poly;
struct Holey_Poly;
struct Spatial_Graph;
//struct poly_group;
typedef std::vector<vec2> Point_Cloud;
struct Monotonic_Zones;
struct Reindexed_Cloud;
struct Intersection;
struct Intersection_List;
enum class Seg_Type;

struct Surface;
struct Sliceable_Group;
struct Spatial_Graph;
struct Triangulation;
struct Id_Line;
//struct graph;
