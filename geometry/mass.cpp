#include "polygon.h"
#include "utils.h"

float geom::area(const Poly& P) {
	float s = 0;
	for(u32 i = 0; i < P.size; i++) {
		s += cross(P[i+1], P[i]);
	}
	return s / 2.f;
}
vec2 geom::area_X_center(const Poly& P) {
	vec2 s = {0.f, 0.f};
	vec2 a, b;
	for(u32 i = 0; i < P.size; i++) {
		a = P[i], b = P[i+1];
		s += cross(b, a) * (a + b);
	}
	return s / 6.f;
}
vec2 geom::mass_center(const Poly& P) {
	return area_X_center(P) / area(P);
}
