#include "demos.h"
#include "plot.h"
#include "spatial_graph.h"

void demo::spatial_graph_intersections() {
	bool pressed = false;
	using namespace geom;

	float win_size = 800;
	box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();

	sf::Clock clock;
	auto get_frame = [&clock] (u32 n) { return ((u32)(s32)(clock.getElapsedTime().asSeconds() * 2.f)) % (n + 1); };

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plotter plt(window);
	std::vector<std::vector<vec2>> lines = {{}};
	spatial_graph SG;
	point_cloud PC;

	auto convert_to_graph = [&SG, &lines] () -> void {
		u32 V = 0;
		for(auto& l: lines) { 
			u32 v = l.size();
			if (v > 1) V += v;
		}
		SG.verts.resize(V), SG.edges.resize(V);
		for(u32 i = 0, I = lines.size(), s = 0; i < I; i++) {
			u32 J = lines[i].size();
			if(J < 2) continue;
			for(u32 j = 0; j < J; j++) { SG.verts[s + j] = lines[i][j]; }
			for(u32 j = 1; j < J - 1; j++) { SG.edges[s + j] = {s + j - 1, s + j + 1}; }
			SG.edges[s] = {s + 1}, SG.edges[s + J - 1] = {s + J - 2};
			s += J;
		}
	};

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
					(--lines.end())->push_back(mpos);
					convert_to_graph();
					break;
				}
				case sf::Event::KeyPressed: {
					if(e.key.code == sf::Keyboard::L) {
						if(lines.size() && (--lines.end())->size() > 1) lines.push_back({});
					} else if(e.key.code == sf::Keyboard::Enter) {
						//PC = add_self_intersections(SG);
					}
				}
			}
		}
		// if(pressed) {
			
		// }
		window.clear();

		plt->draw(SG);
		plt->draw(PC);
		window.display();
	}
}
