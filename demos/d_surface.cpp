#include "demos.h"
#include "polygon.h"
#include "plot.h"
#include "primitives.h"
#include "surface.h"
#include "sliceable_group.h"

void demo::surface() {
	bool pressed = false;
	using namespace geom;

	float point_rad = 3, win_size = 800;
	Box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plotter plt(window);

	Surface S({800, 800}, {1, 1});
	Sliceable_Group SG;

	Poly P = {};
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
					break;
				}
				case sf::Event::KeyPressed: {
					if(e.key.code == sf::Keyboard::D) {
						//printf("->D\n");
						SG.update(pressed, Sliceable_Group::DRAW, mpos);
					} else if(e.key.code == sf::Keyboard::M) {
						//printf("->M\n");
						SG.update(pressed, Sliceable_Group::MOVE, mpos);
					} else if(e.key.code == sf::Keyboard::K) {
						SG.update(pressed, Sliceable_Group::SLICE, mpos);						
					}
				}
			}
		}

		//if(I++ % 300 == 0)
		SG.update(pressed, Sliceable_Group::NONE, mpos);

		S.clear();
		S.draw(SG);
		//printf("polys count: %d\n", SG.polys.size());

		window.clear();
		plt->draw(S);
		window.display();
	}
}
