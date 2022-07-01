#include "polygon.h"

float poly::area() const {
	float s = 0;
	for(u32 i = 0; i < size; i++) {
		s += cross((*this)[i+1], (*this)[i]);
	}
	return s / 2.f;
}
vec2 poly::area_X_center() const {
	vec2 s = {0.f, 0.f};
	vec2 a, b;
	for(u32 i = 0; i < size; i++) {
		a = (*this)[i], b = (*this)[i+1];
		s += cross(b, a) * (a + b);
	}
	return s / 6.f;
}
vec2 poly::mass_center() const {
	return area_X_center() / area();
}
