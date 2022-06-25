#include "demos.h"
#include "polygon.h"
#include "box2.h"
#include <thread>

void demo::ConvexHull() {
	PointCloud cloud = {};

	float pointRad = 3, winSize = 800, centerRad = 10;
	Box2 wintr = {winSize, -winSize, 0, winSize}, inv = wintr.inv();;
	sf::CircleShape point(pointRad);
	point.setOrigin(pointRad, pointRad);

	sf::RenderWindow window(sf::VideoMode(winSize, winSize), "polygons");

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
		//Poly poly = cloud.ToSorted().MakePoly();
		if(cloud.size() > 2) {
			Poly poly = cloud.ToCircularSorted().HullByCircular().MakePoly();
			poly.DrawPoly(window, wintr);
		}
		// auto hull = MinimalHull(cloud);
		// auto poly = MakePoly(cloud, hull);		
		
		for(auto p : cloud) {
			point.setPosition(wintr * p);
			window.draw(point);
		}
		window.display();
	}
}

void demo::ConvexHullTests() {
	float pointRad = 3, winSize = 800, centerRad = 10;
	Box2 wintr = {winSize, -winSize, 0, winSize}, wininv = wintr.inv();

	sf::CircleShape point(pointRad);
	point.setOrigin(pointRad, pointRad);

	sf::RenderWindow window(sf::VideoMode(winSize, winSize), "polygons");

	while (window.isOpen()) {
		for (sf::Event e; window.pollEvent(e);) {
			switch(e.type) {
			case sf::Event::Closed:
				window.close();
				break;
			} 
		}
		auto [res, cloud, poly] = ReindexedCloud::MinimalHullTest(20);
		window.clear(res ? sf::Color(0,0,0) : sf::Color(255,0,0));
		poly.DrawPoly(window, wintr);
		for(auto p : cloud) {
			point.setPosition(wintr * p);
			window.draw(point);
		}
		window.display();
		std::this_thread::sleep_for(std::chrono::seconds(2));
	}
}
