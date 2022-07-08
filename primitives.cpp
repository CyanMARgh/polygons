#include "primitives.h"
#include "polygon.h"

bool circle::inside(vec2 p) const { return len2(p - f1) + len2(p - f2) <= len2(f2 - f1); }
float circle::rad() const { return len(f2 - f1) * .5f; }
vec2 circle::center() const { return (f1 + f2) * .5f; }

s32 cloud_range::size() const { return b - a; }
vec2 cloud_range::at(u32 i) const { return source->at(a + i); }
