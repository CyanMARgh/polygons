#include "polygon.h"

plot::options::options(sf::RenderWindow& rw) {
	this->rw = &rw;
	circle.setRadius(5);
	circle.setOrigin(5, 5);
	float win_size = rw.getSize().y;
	box = {win_size, -win_size, 0, win_size};
}
std::unique_ptr<plot::options> plot::opt = nullptr;
void plot::init(sf::RenderWindow& rw) {
	if(!opt || opt->rw != &rw) opt = std::make_unique<options>(rw);
}
plot::plot(sf::RenderWindow& rw) {
	init(rw);
}
plot::options* plot::operator->() {
	return &*opt;
}

void plot::options::draw_point(vec2 p) {
	circle.setPosition(box * p);
	rw->draw(circle);
}
void plot::options::draw_cloud(const point_cloud& cloud) {
	for(auto p : cloud) {
		draw_point(p);
	}
}
void plot::options::draw_cloud(const reindexed_cloud& cloud) {
	for(auto i : cloud) {
		draw_point(cloud.sat(i));
	}
}
void plot::options::draw_poly(const poly& P) {
	if(P.size < 1) return;
	sf::Vertex *vlines = new sf::Vertex[P.size + 1];
	for(u32 i = 0; i <= P.size; i++) {
		vlines[i] = box * P[i];
	}
	rw->draw(vlines, P.size + 1, sf::LineStrip);
	delete[] vlines;
}

	// static void circle(circle c);
	// static void cloud(const point_cloud& cloud);
	// static void cloud(const reindexed_cloud& cloud);
//};