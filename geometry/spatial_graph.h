#include "polygon.h"
#include <vector>
#include <set>

struct spatial_graph {
	std::vector<std::vector<u32>> edges;
	std::vector<vec2> verts;

	static spatial_graph from_edge_list(const std::vector<id_line>& edges, std::vector<vec2> verts);
};

std::set<id_line> add_self_intersections(spatial_graph& G, std::vector<id_line> marked = {});
// std::vector<poly> extract_inner_polygons(const spatial_graph& G);
spatial_graph graph_union(const spatial_graph& A, const spatial_graph& B);
spatial_graph to_graph(poly P);
void print_graph(const spatial_graph& G);

std::vector<poly> slice_poly(const poly& P, spatial_graph& G);

std::vector<poly> divide_evenly(const poly& P, u32 guide, u32 iterations);