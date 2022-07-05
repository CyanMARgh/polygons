#include "demos.h"
#include "polygon.h"
#include "transforms.h"

void demo::poly_x_line_intersection() {
	enum state_t { 
		FREE, HOLD, NO_LINE,
		FREE_LINE, HOLD_LINE, SLICED
	} state = FREE;

	float point_rad = 3, win_size = 800;
	box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();

	sf::CircleShape point(point_rad);
	point.setOrigin(point_rad, point_rad);

	poly P;
	std::vector<poly> P_sliced;
	line L;
	point_cloud cloud;

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plot plotter(window);

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
				if(e.text.unicode == 10 && P.size > 2) {
					if(state == FREE || state == HOLD) {
						printf("area = %f\n", P.area());
						state = NO_LINE;
					} else if (state == FREE_LINE || state == HOLD_LINE) {
						P_sliced = divide(P, P.find_intersections(L));
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
				} else if(state == HOLD_LINE) {
					state = FREE_LINE;
				}
				break;
			} 
		}
		window.clear();
		switch(state) {
			case HOLD: {
				if(!P.size || mpos != P[-1]) P.add(mpos); 
				break;
			}
			case HOLD_LINE: {
				L.b = mpos;
				cloud = to_cloud(P, P.find_intersections(L));
				break; 
			}
		}
		u32 f_id = ((u32)(s32)(clock.getElapsedTime().asSeconds() * 10));

		if(state == SLICED) {
			if(u32 s = P_sliced.size()) {
				plotter->draw_poly(P_sliced[f_id % s]);
			}
		} else {
			plotter->draw_poly(P);
			//plotter->draw_cloud(cloud);
			if(u32 s = cloud.size()) plotter->draw_point(cloud[f_id % s]);
		}
		window.display();
	}
}
