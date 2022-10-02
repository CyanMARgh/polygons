#include "demos.h"
#include "plot.h"
#include "polygon.h"
#include "holey_polygon.h"
#include "surface.h"
#include "primitives.h"

void demo::holey_poly() {
	using namespace geom;

	bool pressed = false;
	// enum state_t {BASE, POINTS, FINAL} state = BASE;
	float win_size = 800;
	Box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();
	Surface surface({800u, 800u}, {});

	// sf::Clock clock;
	// auto get_frame = [&clock] (u32 n) { return ((u32)(s32)(clock.getElapsedTime().asSeconds() * 20.f)) % (n + 1); };

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plotter plt(window);

	Holey_Poly hp;

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
					if(hp.outer.size) {
						hp.holes.push_back({});
					}
					break;
				}
				case sf::Event::MouseButtonReleased: {
					pressed = false;
					break;
				}
				case sf::Event::KeyPressed: {
				}
			}
		}
		if(pressed) {
			if(hp.holes.size()) {
				Poly& P = hp.holes.back();
				if(!P.size || P[-1] != mpos) P.add(mpos);
			} else {
				Poly& P = hp.outer;
				if(!P.size || P[-1] != mpos) P.add(mpos);
			}
		}
		window.clear();
		surface.clear();
		surface.draw(hp);
		plt->draw(surface);
		window.display();
	}
}
