#include "demos.h"
#include "polygon.h"
#include "transforms.h"

void demo::minimum_circle() {
	point_cloud cloud = {};
	float point_rad = 3, win_size = 800, center_rad = 10, outer_thickness = 2;

	box2 wintr = {win_size,win_size,0,0}, inv = wintr.inv();

	sf::CircleShape point(point_rad), outer_circle(center_rad);
	point.setOrigin(point_rad, point_rad);
	point.setFillColor(sf::Color(255, 255, 255));
	outer_circle.setOutlineThickness(outer_thickness);
	outer_circle.setOutlineColor(sf::Color(255,0,0));
	outer_circle.setFillColor(sf::Color(0,0,0,0));
	outer_circle.setPointCount(300);

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");

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
		for(const auto& p: cloud) {
			point.setPosition(wintr * p);
			window.draw(point);
		}
		circle D;
		// if(cloud.size() == 3) {
		// 	D = trivial({{0, 1, 2}, &cloud});
		// 	D.draw(window, outer_circle, wintr);
		// }
		D = welzl(cloud);
		D.draw(window, outer_circle, wintr);
		window.display();
	}
}