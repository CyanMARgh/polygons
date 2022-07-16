#pragma once

#include "primitives.h"
#include <memory>
#include <SFML/Graphics.hpp>

struct plotter {
	struct base {
		sf::RenderWindow* rw;
		box2 box;
		sf::CircleShape point_spr, circle_spr;
		sf::Sprite any_spr;

		base(sf::RenderWindow& rw);
		void draw(vec2 p);
		void draw(const point_cloud& cloud);
		void draw(const reindexed_cloud& cloud);
		void draw(const poly& P);
		void draw(const circle& C);
		void draw(const surface& S);
	};
	static std::unique_ptr<base> b;
	static void init(sf::RenderWindow& rw);

	plotter(sf::RenderWindow& rw);
	// static void poly(const poly& P);
	// static void circle(circle c);
	base* operator->();
};