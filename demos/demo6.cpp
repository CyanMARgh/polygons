#include "demos.h"
#include "polygon.h"
#include "plot.h"
#include "primitives.h"
#include "surface.h"
#include "sliceable_group.h"

void demo::median_demo() {
	bool pressed = false;
	u32 selected_id = 0;
	float h = 0.f;

	using namespace geom;

	float point_rad = 3, win_size = 800;
	box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plotter plt(window);

	sf::CircleShape point(point_rad);
	point.setOrigin(point_rad, point_rad);
	point.setFillColor(sf::Color(255, 255, 255));

	vec2 A, B, C, D, E, F, Q; 
	auto draw_point = [&point, wintr, &window] (vec2 p) {
		point.setPosition(wintr * p);
		window.draw(point);
	};

	// draw lines on distance h

	while(window.isOpen()) {
		vec2 mpos = inv * (vec2)sf::Mouse::getPosition(window);
		for(sf::Event e; window.pollEvent(e);) {
			switch(e.type) {
				case sf::Event::Closed: {
					window.close();
					break;
				}
				case sf::Event::MouseButtonPressed: {
					pressed = true;
					break;
				}
				case sf::Event::MouseButtonReleased: {
					pressed = false;
					break;
				}
				case sf::Event::KeyPressed: {
					if(e.key.code == sf::Keyboard::Num1) selected_id = 0;
					else if(e.key.code == sf::Keyboard::Num2) selected_id = 1;
					else if(e.key.code == sf::Keyboard::Num3) selected_id = 2;
					else if(e.key.code == sf::Keyboard::Num4) selected_id = 3;
					else if(e.key.code == sf::Keyboard::Num5) selected_id = 4;
					else if(e.key.code == sf::Keyboard::Num6) selected_id = 5;
				}
			}
		}
		if(pressed) {
			switch(selected_id) {
				case 0: A = mpos; break;
				case 1: B = mpos; break;
				case 2: C = mpos; break;
				case 3: D = mpos; break;
				case 4: E = mpos; break;
				case 5: F = mpos; break;
			}
			auto [_h, _Q] = geom::bis_inter({A, B}, {C, D}, {E, F});
			Q = _Q, h = _h;
			//printf("%f %f\n", Q.x, Q.y);
		}

		window.clear();
		plt->draw(A), plt->draw(B), plt->draw(C), plt->draw(D);
		plt->draw(E), plt->draw(F);
		plt->draw({A, B}, plotter::LINE);
		plt->draw({C, D}, plotter::LINE);
		plt->draw({E, F}, plotter::LINE);
		plt->draw(circle::make(Q, h));
		window.display();
	}
}
