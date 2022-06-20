#include "demos.h"

void demo::MassCenterAndSeparation() {
	enum State { BUILD, DONE } state = BUILD;
	struct PointData { vec2 pos; sf::Color color; };
	std::vector<PointData> points = {};

	float pointRad = 3, winSize = 800, centerRad = 10;

	Box2 wintr = {winSize,-winSize,0,winSize}, inv = wintr.inv();

	sf::CircleShape point(pointRad), center(centerRad);
	point.setOrigin(pointRad, pointRad);
	center.setOrigin(centerRad, centerRad);
	center.setFillColor(sf::Color(255, 255, 255));

	Poly poly;
	MonotonicZones mz;

	sf::RenderWindow window(sf::VideoMode(winSize, winSize), "polygons");

	while (window.isOpen()) {
		for (sf::Event e; window.pollEvent(e);) {
			switch(e.type) {
			case sf::Event::Closed:
				window.close();
				break;
			case sf::Event::KeyPressed:
				if(e.text.unicode == 10 && poly.size > 2) {
					state = DONE;
					mz = poly.DivideToMonotonics();
				}
				break;	
			case sf::Event::MouseButtonPressed: 
				vec2 mpos = inv * (vec2)sf::Mouse::getPosition(window);
				if(state == BUILD) poly.add(mpos);
				break;
			} 
		}
		window.clear();
		poly.DrawPoly(window, wintr);
		switch(state) {
		case BUILD:
			if(poly.size > 2) {
				center.setPosition(wintr * poly.MassCenter());
				window.draw(center);
			}
			break;
		case DONE:
			vec2 r = RandVec2();
			points.push_back({wintr * r, poly.IsInsideInt(mz, r) > 0 ? sf::Color(234,182,118) : sf::Color(171,219,227)});
			for(const auto& p: points) {
				point.setFillColor(p.color);
				point.setPosition(p.pos);
				window.draw(point);
			}
			break;
		}
		window.display();
	}
}