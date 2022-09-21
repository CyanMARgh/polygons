#include "demos.h"
#include "polygon.h"
#include "plot.h"
#include "primitives.h"
#include "utils.h"

void demo::mass_center_and_separation() {
	using namespace geom;
	enum State { BUILD, FREE, DONE } state = FREE;
	struct PointData { vec2 pos; sf::Color color; };
	std::vector<PointData> points = {};

	float pointRad = 3, winSize = 800, centerRad = 10;

	Box2 wintr = {winSize,-winSize,0,winSize}, inv = wintr.inv();

	sf::CircleShape point(pointRad), center(centerRad);
	point.setOrigin(pointRad, pointRad);
	center.setOrigin(centerRad, centerRad);
	center.setFillColor(sf::Color(255, 255, 255));

	Poly P;
	Monotonic_Zones mz;

	sf::RenderWindow window(sf::VideoMode(winSize, winSize), "polygons");
	plotter plt(window);

	vec2 n = {.8, .6};

	while (window.isOpen()) {
		for (sf::Event e; window.pollEvent(e);) {
			switch(e.type) {
			case sf::Event::Closed:
				window.close();
				break;
			case sf::Event::KeyPressed:
				if(e.text.unicode == 10 && P.size > 2) {
					state = DONE;
					mz = divide_to_monotonics(P, n);
				}
				break;	
			case sf::Event::MouseButtonPressed: 
				if(state != DONE) {
					state = BUILD;  
				}
				break;
			case sf::Event::MouseButtonReleased:
				if(state != DONE) state = FREE;
				break;
			} 
		}
		window.clear();
		plt->draw(P);
		switch(state) {
			case BUILD: {
				vec2 mpos = inv * (vec2)sf::Mouse::getPosition(window);
				P.add(mpos); 
			}
			case FREE: {
				if(P.size > 2) {
					center.setPosition(wintr * mass_center(P));
					window.draw(center);
				}
				break; 
			}
			case DONE: {
				vec2 r = rand_vec2();
				points.push_back({wintr * r, is_inside_val(P, mz, r, n) > 0 ? sf::Color(234,182,118) : sf::Color(171,219,227)});
				for(const auto& p: points) {
					point.setFillColor(p.color);
					point.setPosition(p.pos);
					window.draw(point);
				}
				break; 
			}
		}
		window.display();
	}
}