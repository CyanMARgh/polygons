#include "geometry.h"

struct triangulation {
	std::vector<std::pair<id_line, id_line>> lines; 
	const point_cloud* source;
};
triangulation make_delaunay_triangulation(const point_cloud& sites_unsorted);
spatial_graph delaunay_to_voronoi(const triangulation& T);
spatial_graph make_voronoi_diagram_2(point_cloud sites);
