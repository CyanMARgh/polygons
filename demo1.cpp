#include "demos.h"
#include "polygon.h"
#include "plot.h"
#include "transforms.h"
#include <thread>

void demo::convex_hull() {
	point_cloud cloud = {};

	float point_rad = 3, win_size = 800, center_rad = 10;
	box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();;
	sf::CircleShape point(point_rad);
	point.setOrigin(point_rad, point_rad);

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plotter plt(window);

	sf::Clock clock;

	enum state_t { FREE, HOLD } state = FREE;
	while (window.isOpen()) {
		vec2 mpos = inv * (vec2)sf::Mouse::getPosition(window);
		for (sf::Event e; window.pollEvent(e);) {
			switch(e.type) {
				case sf::Event::Closed:{
					window.close();
					break;
				}
				case sf::Event::MouseButtonPressed: {
					state = HOLD;
					break;
				}
				case sf::Event::MouseButtonReleased: {
					state = FREE;
					break;
				}
			} 
		}
		if(state == HOLD) cloud.push_back(mpos);
		window.clear();
		if(cloud.size() > 2) {
			poly P = geom::hull_by_circular(geom::to_circular_sorted(cloud)).make_poly();
			plt->draw(P);
		}
		plt->draw(cloud);
		window.display();
	}
}

void demo::convex_hull_tests() {
	float point_rad = 3, win_size = 800, center_rad = 10;
	box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();;

	sf::CircleShape point(point_rad);
	point.setOrigin(point_rad, point_rad);

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plotter plt(window);

	u32 total = 0, failed = 0;

	while (window.isOpen() || total == 10000) {
		for (sf::Event e; window.pollEvent(e);) {
			switch(e.type) {
			case sf::Event::Closed:
				window.close();
				break;
			} 
		}
		auto [res, cloud, P] = geom::minimal_hull_test(50);
		++total, failed += 1 - res; 
		window.clear(res ? sf::Color(0,0,0) : sf::Color(255,0,0));
		plt->draw(P);
		plt->draw(cloud);
		window.display();
		std::this_thread::sleep_for(std::chrono::milliseconds(res ? 0 : 3000));
	}
	printf("%u / %u\n", failed, total);
}
