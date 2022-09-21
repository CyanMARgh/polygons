#include "geometry.h"

struct Triangulation {
	std::vector<std::pair<Id_Line, Id_Line>> lines; 
	const Point_Cloud* source;
};
Triangulation make_delaunay_triangulation(const Point_Cloud& sites_unsorted);
Spatial_Graph delaunay_to_voronoi(const Triangulation& T);
Spatial_Graph make_voronoi_diagram_2(Point_Cloud sites);
