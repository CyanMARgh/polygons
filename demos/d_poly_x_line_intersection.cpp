#include "demos.h"
#include "polygon.h"
#include "plot.h"
#include "primitives.h"
#include "utils.h"

void demo::poly_x_line_intersection() {
	using namespace geom;
	enum state_t { 
		FREE, HOLD, NO_LINE,
		FREE_LINE, HOLD_LINE, SLICED
	} state = FREE;

	float point_rad = 3, win_size = 800;
	Box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();

	sf::CircleShape point(point_rad);
	point.setOrigin(point_rad, point_rad);

	Poly P;
	std::vector<std::pair<Poly, u32>> P_sliced;
	Line L;

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plotter plt(window);

	vec2 mpos_p;

	sf::Clock clock;
	while(window.isOpen()) {
		vec2 mpos = inv * (vec2)sf::Mouse::getPosition(window);

		for(sf::Event e; window.pollEvent(e);) {
			switch(e.type) {
			case sf::Event::Closed: {
				window.close();
				break;
			}
			case sf::Event::KeyPressed: {
				if(e.key.code == sf::Keyboard::K && P.size > 2) {
					if(state == FREE || state == HOLD) {
						state = NO_LINE;
					} 
				} else if(e.key.code == sf::Keyboard::S) {
					if (state == FREE_LINE || state == HOLD_LINE) {
						P_sliced = geom::divide_to_stripes(P, L.a, rrot(normalize(L.b - L.a)), 0.1f, 3);
						state = SLICED;
					}
				}
				break;
			}
			case sf::Event::MouseButtonPressed: {
				if(state == FREE) {
					state = HOLD; 
				} else if(state == FREE_LINE || state == NO_LINE) {
					L.a = mpos;
					state = HOLD_LINE;
				}
				break;
			}
			case sf::Event::MouseButtonReleased:
				if(state == HOLD) {
					state = FREE; 
					if(!P.size || mpos != P[-1]) P.add(mpos); 
				} else if(state == HOLD_LINE) {
					state = FREE_LINE;
				}
				break;
			} 
		}
		window.clear();
		switch(state) {
			case HOLD: {
				//if(!P.size || mpos != P[-1]) P.add(mpos); 
				break;
			}
			case HOLD_LINE: {
				L.b = mpos;
				break; 
			}
		}
		u32 f_id = ((u32)(s32)(clock.getElapsedTime().asSeconds() * 1));

		if(state == SLICED) {
			//printf("here\n");
			if(u32 s = P_sliced.size()) {
				plt->draw(P_sliced[f_id % s].first);
			}
		} else {
			plt->draw(P);
		}
		window.display();
	}
}
