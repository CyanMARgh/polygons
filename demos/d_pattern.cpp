#include "demos.h"
#include "polygon.h"
#include "plot.h"
#include "pattern_maker.h"
#include "utils.h"

void demo::pattern_demo() {
	using namespace geom;

	bool pressed = false;
	enum state_t {PREPARE, FINAL} state = PREPARE;
	float win_size = 800;
	Box2 wintr = {win_size, -win_size, 0, win_size}, inv = wintr.inv();
	sf::Clock clock;

	sf::RenderWindow window(sf::VideoMode(win_size, win_size), "polygons");
	plotter plt(window);

	Poly P;
	std::vector<Poly> P_pattern;
	Pattern pattern = {
		{
			// {Rule::BUFFER, 0.012f, {1}},
			// {Rule::VORONOI, 0.2f, {1}},
			// {Rule::BUFFER, 0.012f, {3}},
			// {Rule::BUFFER, 0.004f, {2}},
			// {Rule::BUFFER, 0.015f, {2}},
			// {Rule::STRIPES, 0.1f, {1}},
			// {Rule::BUFFER, 0.01f, {2}},
			// {Rule::VORONOI, 0.3f, {1}},
			// {Rule::BUFFER, 0.01f, {2}},
			// {Rule::SQUARES, 0.1f, {3}},
			// {Rule::BUFFER, 0.005f, {4}},
			// {Rule::VORONOI, 0.03f, {5}},
			// {Rule::STRIPES, 0.01f, {6}},
			{Rule::SQUARES_SHECKERBOARD, 0.2f, {1, 3}},
			{Rule::BUFFER, 0.01f, {2}},
			{Rule::BUFFER, 0.01f, {2}},
			{Rule::DISTRIBUTION, 0.08f, {4, 1}},
			{Rule::BUFFER, 0.01f, {5}},
			{Rule::END},
		}, 20,
		.placeable_list = {
			Placeable::make(std::vector<vec2>{{0.f, 0.f}, {0.f, 1.f}, {1.f, 0.f}}),
			Placeable::make(std::vector<vec2>{{0.f, 0.f}, {0.f, 1.f}, {1.f, 1.f}, {1.f, 0.f}}),
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
					// if(state == PREPARE) P.add(mpos);		
					break;
				}
				case sf::Event::KeyPressed: {
					if(e.key.code == sf::Keyboard::Enter && state == PREPARE) {
						P_pattern = process(pattern, P);
						state = FINAL;
					}
				}
			}
		}
		if(state == PREPARE && pressed && (!P.size || len(P[-1] - mpos) > 0.05f)) P.add(mpos);		

		window.clear();
		if(state == FINAL) {
			// u32 f_id = ((u32)(s32)(clock.getElapsedTime().asSeconds() * 1.f));
			// u32 v_id = ((u32)(s32)(clock.getElapsedTime().asSeconds() * 10.f));
			// plt->draw(P_pattern[f_id % P_pattern.size()]);
			// plt->draw(P_pattern[f_id % P_pattern.size()][v_id]);
			
			for(auto& Pi : P_pattern) plt->draw(Pi);
		} else {
			plt->draw(P);
		}
		window.display();
	}
}
