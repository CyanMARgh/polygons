#include "polygon.h"
#include <iostream>

int main() {
	enum State {
		BUILD,
		DONE
	} state = BUILD;
	struct PointData {
		vec2 pos;
		sf::Color color;
	};
	std::vector<PointData> points = {};

	float pointRad = 3, winSize = 800, centerRad = 10;
	sf::CircleShape point(pointRad), center(centerRad);
	point.setOrigin(pointRad, pointRad);
	center.setOrigin(centerRad, centerRad);
	center.setFillColor(sf::Color(255, 255, 255));

	Poly poly;
	MonotonicZones mz;

	sf::RenderWindow window(sf::VideoMode(winSize, winSize), "polygons");

	while (window.isOpen()) {
		sf::Event e;
		while (window.pollEvent(e)) {
			switch(e.type) {
			case sf::Event::Closed:
				window.close();
				break;
			case sf::Event::KeyPressed:
				if(e.text.unicode == 10 && poly.size > 2) {
					state = DONE;
					mz = poly.DivideToMonotonics();
					mz.print();
				}
				break;	
			case sf::Event::MouseButtonPressed: 
				vec2 mpos = (vec2)sf::Mouse::getPosition(window) / winSize;
				switch(state) {
				case BUILD:
					poly.add(mpos);
					break;
				case DONE:
					break;
				}
			} 
		}
		window.clear();
		switch(state) {
		case BUILD:
			poly.DrawPoly(window);
			if(poly.size > 2) {
				center.setPosition(poly.MassCenter() * winSize);
				window.draw(center);
			}
			break;
		case DONE:
			vec2 r = RandVec2();
			points.push_back({r * winSize, poly.IsInsideInt(mz, r) > 0 ? sf::Color(234,182,118) : sf::Color(171,219,227)});

			poly.DrawPoly(window);
			for(const auto& p: points) {
				point.setFillColor(p.color);
				point.setPosition(p.pos);
				window.draw(point);
			}
			break;
		}
		window.display();
	}

	return 0;
}