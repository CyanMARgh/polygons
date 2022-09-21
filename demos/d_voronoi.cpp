#include "demos.h"
#include "polygon.h"
#include "plot.h"
#include "primitives.h"
#include "surface.h"
#include "sliceable_group.h"
#include "utils.h"
#include "spatial_graph.h"
#include "voronoi.h"

void demo::voronoi() {
	bool pressed = false;
	using namespace geom;

	float win_size = 800;
	Box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();

	sf::Clock clock;
	auto get_frame = [&clock] (u32 n) {
		return ((u32)(s32)(clock.getElapsedTime().asSeconds() * 2.f)) % (n + 1);
	};

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plotter plt(window);
	Point_Cloud cloud = {};
	Triangulation t = {};
	Spatial_Graph voronoi = {}; 

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
					cloud.push_back(mpos);
					break;
				}
				case sf::Event::KeyPressed: {
					if(e.key.code == sf::Keyboard::Enter) {
						t = make_delaunay_triangulation(cloud);
						voronoi = delaunay_to_voronoi(t);
					} 
				}
			}
		}
		window.clear();

		plt->draw(cloud);
		//plt->draw(t);
		plt->draw(voronoi);
		window.display();
	}
}