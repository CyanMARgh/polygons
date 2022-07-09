#include "plot.h"
#include "polygon.h"
#include "primitives.h"

plotter::base::base(sf::RenderWindow& rw) {
	this->rw = &rw;
	point_spr.setRadius(5);
	point_spr.setOrigin(5, 5);
	circle_spr.setOutlineThickness(2);
	circle_spr.setOutlineColor(sf::Color::Red);
	circle_spr.setFillColor(sf::Color(0,0,0,0));
	circle_spr.setPointCount(400);
	float win_size = rw.getSize().y;
	box = {win_size, -win_size, 0, win_size};
}
std::unique_ptr<plotter::base> plotter::b = nullptr;
void plotter::init(sf::RenderWindow& rw) {
	if(!b || b->rw != &rw) b = std::make_unique<base>(rw);
}
plotter::plotter(sf::RenderWindow& rw) {
	init(rw);
}
plotter::base* plotter::operator->() {
	return &*b;
}

void plotter::base::draw(vec2 p) {
	point_spr.setPosition(box * p);
	rw->draw(point_spr);
}
void plotter::base::draw(const circle& c) {
	circle_spr.setPosition(box * c.center());
	float r = c.rad() * box.s.y;
	circle_spr.setRadius(r);
	circle_spr.setOrigin(r, r);
	rw->draw(circle_spr);
}
void plotter::base::draw(const point_cloud& cloud) {
	for(auto p : cloud) {
		draw(p);
	}
}
void plotter::base::draw(const reindexed_cloud& cloud) {
	for(auto i : cloud) {
		draw(cloud.sat(i));
	}
}
void plotter::base::draw(const poly& P) {
	if(P.size < 1) return;
	sf::Vertex *vlines = new sf::Vertex[P.size + 1];
	for(u32 i = 0; i <= P.size; i++) {
		vlines[i] = box * P[i];
	}
	rw->draw(vlines, P.size + 1, sf::LineStrip);
	delete[] vlines;
}