#include "demos.h"
#include "plot.h"
#include "spatial_graph.h"
#include "utils.h"
#include "voronoi.h"

//DEMO TO HEADERS
void demo::even_slicing() {
	const u32 guide = 50;
	const float iteration_delay = .2f;
	using namespace geom;

	bool pressed = false;
	enum state_t {BASE, FINAL} state = BASE;
	float win_size = 800;
	Box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();

	sf::Clock clock;

	
	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plotter plt(window);

	Poly P;
	std::vector<Poly> P_sliced;
	Point_Cloud sites;
	Box2 box;
	auto random_point = [&box] { return box * rand_vec2(); };

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
					// if(state == BASE) {
					// 	if(!P.size || P[-1] != mpos) P.add(mpos);
					// }
					break;
				}
				case sf::Event::KeyPressed: {
					if(e.key.code == sf::Keyboard::Enter) {
						if(state == BASE && P.size > 2) {
							state = FINAL;
							sites.resize(guide);
							//auto voronoi_graph = delaunay_to_voronoi(make_delaunay_triangulation(sites));
							auto voronoi_graph = make_voronoi_diagram_2(sites);
							P_sliced = slice_poly(P, voronoi_graph);
							Box2 box = P.bounding_box();
						}
					}
				}
			}
		}
		if(state == FINAL && clock.getElapsedTime().asSeconds() > iteration_delay) {
			clock.restart();
			u32 n = P_sliced.size();
			for(u32 j = 0; j < n && j < guide; j++) sites[j] = geom::mass_center(P_sliced[j]);
			for(u32 j = n; j < guide; j++) sites[j] = random_point();
			//auto voronoi_graph = delaunay_to_voronoi(make_delaunay_triangulation(sites));
			auto voronoi_graph = make_voronoi_diagram_2(sites);
			P_sliced = slice_poly(P, voronoi_graph);
		}
		if(state == BASE && pressed) {
			if(!P.size || P[-1] != mpos) P.add(mpos);
		}

		window.clear();
		plt->draw(sites);
		plt->draw(P);
		if(state == FINAL) {
			//if(P_sliced.size())plt->draw(P_sliced[get_frame(P_sliced.size()-1)]);
			for(auto& Pi : P_sliced) {
				plt->draw(Pi);
			}
		}
		window.display();
	}
}
