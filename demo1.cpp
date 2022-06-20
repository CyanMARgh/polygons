#include "demos.h"

void demo::ConvexHull() {
	PointCloud cloud = {};

	Box2 wintr = {800, 800};
	float pointRad = 3, winSize = 800, centerRad = 10;
	sf::CircleShape point(pointRad);
	point.setOrigin(pointRad, pointRad);

	sf::RenderWindow window(sf::VideoMode(winSize, winSize), "polygons");

	while (window.isOpen()) {
		for (sf::Event e; window.pollEvent(e);) {
			switch(e.type) {
			case sf::Event::Closed:
				window.close();
				break;
			case sf::Event::MouseButtonPressed: 
				vec2 mpos = (vec2)sf::Mouse::getPosition(window) / winSize;
				cloud.push_back(mpos);
				break;
			} 
		}
		window.clear();
		auto hull = MinimalHull(cloud);
		auto poly = MakePoly(cloud, hull);		
		//auto poly = SortedPoly(cloud);
		poly.DrawPoly(window, wintr);
		for(auto p : cloud) {
			point.setPosition(p * winSize);
			window.draw(point);
		}
		window.display();
	}
}