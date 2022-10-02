#include "geometry.h"
#include "polygon.h"
#include <vector>

struct Holey_Poly {
	Poly outer;
	std::vector<Poly> holes;
};
