#include "plot.h"
#include "polygon.h"
#include "primitives.h"
#include "surface.h"
#include <string.h>
#include "utils.h"

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
void plotter::base::draw(line L, style s) {
	sf::Vertex vlines[2];
	if(s == LINE) {
		vec2 v = L.b - L.a;
		L = {(L.a - v * 100.f), (L.b + v * 100.f)};
	}
	L = {box * L.a, box * L.b};
	vlines[0] = L.a;
	vlines[1] = L.b;
	rw->draw(vlines, 2, sf::LineStrip);
}
void plotter::base::draw(const surface& S) {
	vec2u size = S.size;
	sf::Image img;
	img.create(size.x, size.y);
	u32* imgdata = (u32*)img.getPixelsPtr();
	for(u32 y = 0, i = 0; y < size.y; y++) {
		for(u32 x = 0; x < size.x; x++, i++) {
			imgdata[i] = S.data[x + (size.y - 1 - y) * size.x];
		}
	}
	sf::Texture tex;
	tex.loadFromImage(img);
	any_spr.setTexture(tex);
	any_spr.setScale((vec2)rw->getSize() / (vec2)S.size);
	rw->draw(any_spr);
}
void plotter::base::draw(const geom::triangulation& T) {
	for(auto t : T.lines) {
		vec2 A = T.source->at(t.a), B = T.source->at(t.b);
		draw({A, B}, style::SEGMENT);
	}
}