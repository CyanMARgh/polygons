#include "demos.h"
#include "polygon.h"
#include "plot.h"
#include "primitives.h"
#include "surface.h"
#include "sliceable_group.h"
#include "utils.h"
#include "buffer.h"

void demo::skeleton() {
	bool pressed = false;

	using namespace geom;

	float win_size = 800;
	Box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();

	sf::Clock clock;

	auto get_frame = [&clock] (u32 n) {
		return ((u32)(s32)(clock.getElapsedTime().asSeconds() * 2.f)) % (n + 1);
	};

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plotter plt(window);
	Poly P;
	std::vector<Poly> P_buffered;
	float h = 0.f;
	float dh = 0.002f;
	u32 I = 0;

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
					//P.add(mpos);
					break;
				}
				case sf::Event::KeyPressed: {
					if(e.key.code == sf::Keyboard::Up) {
						h += dh;
						P_buffered = geom::buffer(P, h);
					} else if(e.key.code == sf::Keyboard::Down) {
						h -= dh; //if(h < 0.f) h = 0.f; 
						P_buffered = geom::buffer(P, h);
					} 
				}
			}
		}
		if(pressed && (!P.size || P[-1] != mpos) && (++I % 300 == 0)) P.add(mpos + rand_vec2() * 1.e-5f);
		window.clear();

		plt->draw(P);
		//printf("%lu\n", P_buffered.size());
		for(auto Pi : P_buffered) {
			plt->draw(Pi);			
		}
		window.display();
	}
}
