#include "sliceable_group.h"
#include "polygon.h"
#include "primitives.h"
#include "utils.h"
#include "color.h"

Poly* Sliceable_Group::top() {
	u32 s = order.size();
	return s ? &polys[order[s - 1]] : nullptr;
}
const Poly* Sliceable_Group::top() const {
	u32 s = order.size();
	return s ? &polys[order[s - 1]] : nullptr;
}

void Sliceable_Group::start_draw() {
	u32 s = polys.size();
	Poly np = {};
	polys.push_back(np);
	order.push_back(s);
	colors.push_back(color::lazy_any(s));
}
void Sliceable_Group::keep_draw() {
	Poly &T = *top();
	if(!T.size || T[-1] != mouse_curr) T.add(mouse_curr);
}
void Sliceable_Group::end_draw() {
	Poly* T = top();
	if(!T) throw std::runtime_error("no top Poly\n");
	if(geom::is_valid(*T)) {
		mzs.push_back(geom::divide_to_monotonics(*T));
	} else {
		//printf("not valid!\n");
		polys.pop_back();
		colors.pop_back();
		order.pop_back();
	}
}


void Sliceable_Group::start_move() {
	mouse_buf = mouse_curr;
	for(s32 i = order.size() - 1, j; i >= 0; i--) {
		j = order[i];
		if(geom::is_inside_val(polys[j], mzs[j], mouse_curr)) {
			std::rotate(order.begin() + i, order.begin() + i + 1, order.end());
			selected = true;
			return;
		}
	}
}
void Sliceable_Group::keep_move() { }
void Sliceable_Group::end_move() {
	if(selected) {
		vec2 delta = mouse_curr - mouse_buf;
		//printf("delta: (%f %f)\n", delta.x, delta.y);
		Poly* T = top();
		if(T) {
			for(u32 i = 0, n = T->size; i < n; i++) {
				(*T)[i] += delta;
			}
		}
	}
	selected = false;
	mouse_buf = {0, 0};
}


void Sliceable_Group::start_slice() { mouse_buf = mouse_curr; }
void Sliceable_Group::keep_slice() { }
void Sliceable_Group::end_slice() {
	Line L = {mouse_buf, mouse_curr};
	if(len(L.a - L.b) < EPS) L.b = L.a + vec2(0., 1.);
	Poly& T = *top();
	//std::vector<Poly> p_sliced = divide(T, geom::find_intersections(T, L));
	//std::vector<Poly> p_sliced = geom::divide(T, L.a, rrot(normalize(L.b - L.a)), 0.1f, 3);
	std::vector<std::tuple<Poly, u32, u32>> p_sliced = geom::divide_to_squares(T, normalize(L.b - L.a), 0.1f);

	auto make_color = [] (std::tuple<Poly, u32, u32>& Q) -> u32 { return mmod((std::get<1>(Q) + std::get<2>(Q)), 2) ? 0xFF00FFFF : 0xFFFF00FF; };

	u32 s = order.size(), n = p_sliced.size();

	mzs[order[s - 1]] = geom::divide_to_monotonics(std::get<0>(p_sliced[0]));
	polys[order[s - 1]] = std::get<0>(p_sliced[0]);
	colors[order[s - 1]] = make_color(p_sliced[0]);

	for(u32 i = 1; i < n; i++) {
		mzs.push_back(geom::divide_to_monotonics(std::get<0>(p_sliced[i])));
		auto& psi = p_sliced[i];
		polys.push_back(std::get<0>(psi));
		order.push_back(s + i - 1);
		// colors.push_back(color::lazy_any(colors.size()));
		colors.push_back(make_color(psi));
	}
}


void Sliceable_Group::update(bool pressed_new, mode_t mode_new, vec2 mouse_new) {
	mouse_curr = mouse_new;
	if(pressed && pressed_new) {
		if(mode == DRAW) {
			keep_draw();
		} else if (mode == MOVE) {
			keep_move();
		} else if (mode == SLICE) {
			keep_slice();
		}
	} else if(pressed) {
		if(mode == DRAW) {
			end_draw();
		} else if(mode == MOVE) {
			end_move();
		} else if(mode == SLICE) {
			end_slice();
		}
	} else if(pressed_new) {
		if(mode == DRAW) {
			start_draw();
		} else if(mode == MOVE) {
			start_move();
		} else if(mode == SLICE) {
			start_slice();
		}
	} else {
		if(mode_new != NONE) mode = mode_new;
	}
	pressed = pressed_new;
}

