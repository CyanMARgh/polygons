#include "color.h"
#include <cmath>
// vec3 color::rgb2hsv(vec3 c) {
	
// }
vec3 color::hsv2rgb(vec3 in) {
	double hh, p, q, t, ff;
	long i;
	vec3 out;

	if (in.y <= 0.0) {       // < is bogus, just shuts up warnings
		out.x = in.z;
		out.y = in.z;
		out.z = in.z;
		return out;
	}
	hh = in.x;
	if (hh >= 360.0) hh = 0.0;
	hh /= 60.0;
	i = (long)hh;
	ff = hh - i;
	p = in.z * (1.0 - in.y);
	q = in.z * (1.0 - (in.y * ff));
	t = in.z * (1.0 - (in.y * (1.0 - ff)));

	switch (i) {
		case 0: out.x = in.z;
			out.y = t;
			out.z = p;
			break;
		case 1: out.x = q;
			out.y = in.z;
			out.z = p;
			break;
		case 2: out.x = p;
			out.y = in.z;
			out.z = t;
			break;

		case 3: out.x = p;
			out.y = q;
			out.z = in.z;
			break;
		case 4: out.x = t;
			out.y = p;
			out.z = in.z;
			break;
		case 5:
		default: out.x = in.z;
			out.y = p;
			out.z = q;
			break;
	}
	return out;
}
u32 float_to_u8(float x) {
	s32 ix = x * 255;
	return ix > 255 ? 255 : ix < 0 ? 0 : ix;
}
u32 color::rgb_code(vec3 c) {
	return (float_to_u8(c.x) << 16) | (float_to_u8(c.y) << 8) | float_to_u8(c.z) | 0xff000000;
	//return 0xff00ff00;
}
u32 color::lazy_any(u32 i) {
	vec3 col = vec3(
			fmod(1.61803399f * i, 1.f) * 360.f,
			1.f,//fmod(3.14159265f * i, 1.f) * .8f + .2f,
			1.f
	);
	return rgb_code(hsv2rgb(col));
}