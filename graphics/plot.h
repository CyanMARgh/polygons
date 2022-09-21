#pragma once

#include "primitives.h"
#include <memory>
#include <SFML/Graphics.hpp>
#include "polygon.h"

struct plotter {
	enum style {
		LINE,
		SEGMENT
	};
	struct base {
		sf::RenderWindow* rw;
		Box2 box;
		sf::CircleShape point_spr, circle_spr;
		sf::Sprite any_spr;

		base(sf::RenderWindow& rw);
		void draw(vec2 p);
		void draw(const Point_Cloud& cloud);
		void draw(const Reindexed_Cloud& cloud);
		void draw(const Poly& P);
		void draw(const Circle& C);
		void draw(const Surface& S);
		void draw(Line L, style s = LINE);
		void draw(const Triangulation& T);
		void draw(const Spatial_Graph& T);
	};
	static std::unique_ptr<base> b;
	static void init(sf::RenderWindow& rw);

	plotter(sf::RenderWindow& rw);
	// static void Poly(const Poly& P);
	// static void Circle(Circle c);
	base* operator->();
};