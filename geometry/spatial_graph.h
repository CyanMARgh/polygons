#include "polygon.h"
#include <vector>
#include <set>

struct Spatial_Graph {
	std::vector<std::vector<u32>> edges;
	std::vector<vec2> verts;

	static Spatial_Graph from_edge_list(const std::vector<Id_Line>& edges, std::vector<vec2> verts);
};

std::set<Id_Line> add_self_intersections(Spatial_Graph& G, std::vector<Id_Line> marked = {});
// std::vector<Poly> extract_inner_polygons(const Spatial_Graph& G);
Spatial_Graph graph_union(const Spatial_Graph& A, const Spatial_Graph& B);
Spatial_Graph to_graph(Poly P);
void print_graph(const Spatial_Graph& G);

std::vector<Poly> slice_poly(const Poly& P, Spatial_Graph& G);

std::pair<std::vector<Poly>, std::vector<vec2>> divide_evenly(const Poly& P, u32 guide, u32 iterations);