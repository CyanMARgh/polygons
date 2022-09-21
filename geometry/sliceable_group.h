#include "geometry.h"

struct Sliceable_Group {
	std::vector<Poly> polys = {};
	std::vector<Monotonic_Zones> mzs = {};
	std::vector<u32> colors = {};
	std::vector<u32> order = {};

	vec2 mouse_buf = {0.f ,0.f}, mouse_curr = {0.f ,0.f};
	enum mode_t { DRAW, MOVE, SLICE, NONE } mode = NONE;
	bool pressed = false, selected = false; 

	void update(bool pressed_new, mode_t mode_new, vec2 mouse_new);

	Poly* top();
	const Poly* top() const;

	void start_draw();
	void keep_draw();
	void end_draw();

	void start_move();
	void keep_move();
	void end_move();

	void start_slice();
	void keep_slice();
	void end_slice();
};