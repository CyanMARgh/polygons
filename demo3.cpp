#include "demos.h"
#include "polygon.h"
#include "transforms.h"

void demo::minimum_circle() {
	point_cloud cloud = {};
	float point_rad = 3, win_size = 800, center_rad = 10, outer_thickness = 2;

	box2 wintr = {win_size, -win_size,0,win_size}, inv = wintr.inv();

	sf::CircleShape outer_circle(center_rad);
	outer_circle.setOutlineThickness(outer_thickness);
	outer_circle.setOutlineColor(sf::Color(255,0,0));
	outer_circle.setFillColor(sf::Color(0,0,0,0));
	outer_circle.setPointCount(300);

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plot plotter(window);

	enum state_t { FREE, HOLD } state = FREE;
	while (window.isOpen()) {
		vec2 mpos = inv * (vec2)sf::Mouse::getPosition(window);
		for (sf::Event e; window.pollEvent(e);) {
			switch(e.type) {
				case sf::Event::Closed:
					window.close();
					break;
				case sf::Event::MouseButtonPressed: {
					cloud.push_back(mpos);
					state = HOLD;
					break;
				}
				case sf::Event::MouseButtonReleased: {
					state = FREE;
					break;
				}
			} 
		}
		//if(state == HOLD) cloud.push_back(mpos);
		window.clear();
		plotter->draw_cloud(cloud);
		welzl::get(cloud).draw(window, outer_circle, wintr);
		window.display();
	}
}