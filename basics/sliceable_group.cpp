#include "sliceable_group.h"
#include "polygon.h"
#include "primitives.h"
#include "utils.h"
#include "color.h"

poly* sliceable_group::top() {
	u32 s = order.size();
	return s ? &polys[order[s - 1]] : nullptr;
}
const poly* sliceable_group::top() const {
	u32 s = order.size();
	return s ? &polys[order[s - 1]] : nullptr;
}

void sliceable_group::start_draw() {
	u32 s = polys.size();
	poly np = {};
	polys.push_back(np);
	order.push_back(s);
	colors.push_back(color::lazy_any(s));
}
void sliceable_group::keep_draw() {
	poly &T = *top();
	if(!T.size || T[-1] != mouse_curr) T.add(mouse_curr);
}
void sliceable_group::end_draw() {
	poly* T = top();
	if(!T) throw std::runtime_error("no top poly\n");
	if(geom::is_valid(*T)) {
		mzs.push_back(geom::divide_to_monotonics(*T));
	} else {
		//printf("not valid!\n");
		polys.pop_back();
		colors.pop_back();
		order.pop_back();
	}
}


void sliceable_group::start_move() {
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
void sliceable_group::keep_move() { }
void sliceable_group::end_move() {
	if(selected) {
		vec2 delta = mouse_curr - mouse_buf;
		//printf("delta: (%f %f)\n", delta.x, delta.y);
		poly* T = top();
		if(T) {
			for(u32 i = 0, n = T->size; i < n; i++) {
				(*T)[i] += delta;
			}
		}
	}
	selected = false;
	mouse_buf = {0, 0};
}


void sliceable_group::start_slice() { mouse_buf = mouse_curr; }
void sliceable_group::keep_slice() { }
void sliceable_group::end_slice() {
	line L = {mouse_buf, mouse_curr};
	if(len(L.a - L.b) < EPS) L.b = L.a + vec2(0., 1.);
	poly& T = *top();
	std::vector<poly> p_sliced = divide(T, geom::find_intersections(T, L));
	u32 s = order.size(), n = p_sliced.size();

	mzs[order[s - 1]] = geom::divide_to_monotonics(p_sliced[0]);
	polys[order[s - 1]] = p_sliced[0];

	for(u32 i = 1; i < n; i++) {
		mzs.push_back(geom::divide_to_monotonics(p_sliced[i]));
		polys.push_back(p_sliced[i]);
		order.push_back(s + i - 1);
		colors.push_back(color::lazy_any(colors.size()));
	}
}


void sliceable_group::update(bool pressed_new, mode_t mode_new, vec2 mouse_new) {
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

