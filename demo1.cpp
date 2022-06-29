#include "demos.h"
#include "polygon.h"
#include "transforms.h"
#include <thread>

void demo::convex_hull() {
	point_cloud cloud = {};

	float point_rad = 3, win_size = 800, center_rad = 10;
	box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();;
	sf::CircleShape point(point_rad);
	point.setOrigin(point_rad, point_rad);

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");

	sf::Clock clock;

	while (window.isOpen()) {
		for (sf::Event e; window.pollEvent(e);) {
			switch(e.type) {
			case sf::Event::Closed:
				window.close();
				break;
			case sf::Event::MouseButtonPressed: 
				vec2 mpos = inv * (vec2)sf::Mouse::getPosition(window);
				cloud.push_back(mpos);
				break;
			} 
		}
		window.clear();
		if(cloud.size() > 2) {
			poly P = cloud.to_circular_sorted().hull_by_circular().make_poly();
			P.draw(window, wintr);
		}
		
		for(auto p : cloud) {
			point.setPosition(wintr * p);
			window.draw(point);
		}
		window.display();
	}
}

void demo::convex_hull_tests() {
	float point_rad = 3, win_size = 800, center_rad = 10;
	box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();;

	sf::CircleShape point(point_rad);
	point.setOrigin(point_rad, point_rad);

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");

	u32 total = 0, failed = 0;

	while (window.isOpen() || total == 10000) {
		for (sf::Event e; window.pollEvent(e);) {
			switch(e.type) {
			case sf::Event::Closed:
				window.close();
				break;
			} 
		}
		auto [res, cloud, P] = reindexed_cloud::minimal_hull_test(50);
		++total, failed += 1 - res; 
		window.clear(res ? sf::Color(0,0,0) : sf::Color(255,0,0));
		P.draw(window, wintr);
		for(auto p : cloud) {
			point.setPosition(wintr * p);
			window.draw(point);
		}
		window.display();
		std::this_thread::sleep_for(std::chrono::milliseconds(res ? 0 : 3000));
	}
	printf("%u / %u\n", failed, total);
}
