#include "demos.h"
#include "plot.h"
#include "spatial_graph.h"
#include "voronoi.h"

void demo::voronoi_slicing() {
	using namespace geom;

	bool pressed = false;
	enum state_t {BASE, POINTS, FINAL} state = BASE;
	float win_size = 800;
	box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();

	sf::Clock clock;
	auto get_frame = [&clock] (u32 n) { return ((u32)(s32)(clock.getElapsedTime().asSeconds() * 20.f)) % (n + 1); };

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plotter plt(window);

	poly P;
	std::vector<poly> P_sliced;
	spatial_graph voronoi_graph;
	point_cloud sites;

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
					if(state == BASE) {
						
					} else if(state == POINTS) {
						sites.push_back(mpos);
					}
					break;
				}
				case sf::Event::KeyPressed: {
					if(e.key.code == sf::Keyboard::Enter) {
						if(state == BASE) {
							if(P.size > 2) state = POINTS;
						} else if(state == POINTS) {
							//printf("-> %d\n", make_voronoi_diagram_2(sites));
							voronoi_graph = make_voronoi_diagram_2(sites);
							printf("(0)\n");
							//voronoi_graph = delaunay_to_voronoi(make_delaunay_triangulation(sites));
							// voronoi_graph = graph_union(to_graph(P), voronoi_graph);
							// add_self_intersections(voronoi_graph);
							//P_sliced = slice_poly(P, voronoi_graph);
							state = FINAL;
						}
					}
				}
			}
		}
		if(pressed && state == BASE) {
			if(!P.size || P[-1] != mpos) P.add(mpos);
		}
		window.clear();
		plt->draw(sites);
		plt->draw(P);
		if(state == FINAL) {
			//if(P_sliced.size())plt->draw(P_sliced[get_frame(P_sliced.size()-1)]);
			plt->draw(voronoi_graph);
		}
		//for(const auto& Pi : P_sliced) plt->draw(Pi);
		window.display();
	}
}
