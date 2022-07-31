FLAGS=-lsfml-window -lsfml-system -lsfml-graphics -lpthread -std=c++17 -lm -O3
all: build/app

build/app: tmp/graphics_color.o tmp/graphics_plot.o tmp/demos_demo5.o tmp/demos_demo2.o tmp/demos_demo3.o tmp/demos_demo6.o tmp/demos_demo0.o tmp/demos_demo1.o tmp/demos_demo4.o tmp/tree_tree.o tmp/basics_utils.o tmp/basics_sliceable_group.o tmp/basics_primitives.o tmp/geometry_surface.o tmp/geometry_polygon.o tmp/geometry_serialization.o tmp/geometry_mass.o tmp/geometry_hull.o tmp/main.o
	g++ -o build/app tmp/graphics_color.o tmp/graphics_plot.o tmp/demos_demo5.o tmp/demos_demo2.o tmp/demos_demo3.o tmp/demos_demo6.o tmp/demos_demo0.o tmp/demos_demo1.o tmp/demos_demo4.o tmp/tree_tree.o tmp/basics_utils.o tmp/basics_sliceable_group.o tmp/basics_primitives.o tmp/geometry_surface.o tmp/geometry_polygon.o tmp/geometry_serialization.o tmp/geometry_mass.o tmp/geometry_hull.o tmp/main.o $(FLAGS)
tmp/graphics_color.o: ./graphics/color.cpp tmp/i ./basics/geometry.h ./graphics/color.h
	g++ -c ./graphics/color.cpp -o tmp/graphics_color.o -I ./basics/ -I ./graphics/ $(FLAGS)
tmp/graphics_plot.o: ./graphics/plot.cpp tmp/i ./basics/utils.h ./basics/geometry.h ./basics/primitives.h ./graphics/plot.h ./geometry/surface.h ./geometry/polygon.h
	g++ -c ./graphics/plot.cpp -o tmp/graphics_plot.o -I ./basics/ -I ./graphics/ -I ./geometry/ $(FLAGS)
tmp/demos_demo5.o: ./demos/demo5.cpp tmp/i ./demos/demos.h ./basics/sliceable_group.h ./basics/geometry.h ./basics/primitives.h ./graphics/plot.h ./geometry/surface.h ./geometry/polygon.h
	g++ -c ./demos/demo5.cpp -o tmp/demos_demo5.o -I ./basics/ -I ./graphics/ -I ./demos/ -I ./geometry/ $(FLAGS)
tmp/demos_demo2.o: ./demos/demo2.cpp tmp/i ./basics/utils.h ./demos/demos.h ./tree/tree.h ./basics/geometry.h
	g++ -c ./demos/demo2.cpp -o tmp/demos_demo2.o -I ./basics/ -I ./demos/ -I ./tree/ $(FLAGS)
tmp/demos_demo3.o: ./demos/demo3.cpp tmp/i ./graphics/plot.h ./basics/geometry.h ./basics/primitives.h ./demos/demos.h ./geometry/polygon.h
	g++ -c ./demos/demo3.cpp -o tmp/demos_demo3.o -I ./basics/ -I ./graphics/ -I ./demos/ -I ./geometry/ $(FLAGS)
tmp/demos_demo6.o: ./demos/demo6.cpp tmp/i ./demos/demos.h ./basics/sliceable_group.h ./basics/geometry.h ./basics/primitives.h ./graphics/plot.h ./geometry/surface.h ./geometry/polygon.h
	g++ -c ./demos/demo6.cpp -o tmp/demos_demo6.o -I ./basics/ -I ./graphics/ -I ./demos/ -I ./geometry/ $(FLAGS)
tmp/demos_demo0.o: ./demos/demo0.cpp tmp/i ./basics/utils.h ./demos/demos.h ./basics/geometry.h ./basics/primitives.h ./graphics/plot.h ./geometry/polygon.h
	g++ -c ./demos/demo0.cpp -o tmp/demos_demo0.o -I ./basics/ -I ./graphics/ -I ./demos/ -I ./geometry/ $(FLAGS)
tmp/demos_demo1.o: ./demos/demo1.cpp tmp/i ./graphics/plot.h ./basics/geometry.h ./basics/primitives.h ./demos/demos.h ./geometry/polygon.h
	g++ -c ./demos/demo1.cpp -o tmp/demos_demo1.o -I ./basics/ -I ./graphics/ -I ./demos/ -I ./geometry/ $(FLAGS)
tmp/demos_demo4.o: ./demos/demo4.cpp tmp/i ./graphics/plot.h ./basics/geometry.h ./basics/primitives.h ./demos/demos.h ./geometry/polygon.h
	g++ -c ./demos/demo4.cpp -o tmp/demos_demo4.o -I ./basics/ -I ./graphics/ -I ./demos/ -I ./geometry/ $(FLAGS)
tmp/tree_tree.o: ./tree/tree.cpp tmp/i ./basics/utils.h ./tree/tree.h ./basics/geometry.h
	g++ -c ./tree/tree.cpp -o tmp/tree_tree.o -I ./basics/ -I ./tree/ $(FLAGS)
tmp/basics_utils.o: ./basics/utils.cpp tmp/i ./basics/utils.h ./basics/primitives.h ./basics/geometry.h
	g++ -c ./basics/utils.cpp -o tmp/basics_utils.o -I ./basics/ $(FLAGS)
tmp/basics_sliceable_group.o: ./basics/sliceable_group.cpp tmp/i ./basics/utils.h ./graphics/color.h ./basics/sliceable_group.h ./basics/geometry.h ./basics/primitives.h ./geometry/polygon.h
	g++ -c ./basics/sliceable_group.cpp -o tmp/basics_sliceable_group.o -I ./basics/ -I ./graphics/ -I ./geometry/ $(FLAGS)
tmp/basics_primitives.o: ./basics/primitives.cpp tmp/i ./basics/utils.h ./basics/primitives.h ./basics/geometry.h
	g++ -c ./basics/primitives.cpp -o tmp/basics_primitives.o -I ./basics/ $(FLAGS)
tmp/geometry_surface.o: ./geometry/surface.cpp tmp/i ./basics/utils.h ./basics/sliceable_group.h ./basics/geometry.h ./basics/primitives.h ./geometry/surface.h ./geometry/polygon.h
	g++ -c ./geometry/surface.cpp -o tmp/geometry_surface.o -I ./basics/ -I ./geometry/ $(FLAGS)
tmp/geometry_polygon.o: ./geometry/polygon.cpp tmp/i ./basics/utils.h ./basics/geometry.h ./basics/primitives.h ./geometry/polygon.h
	g++ -c ./geometry/polygon.cpp -o tmp/geometry_polygon.o -I ./basics/ -I ./geometry/ $(FLAGS)
tmp/geometry_serialization.o: ./geometry/serialization.cpp tmp/i ./basics/geometry.h ./geometry/polygon.h
	g++ -c ./geometry/serialization.cpp -o tmp/geometry_serialization.o -I ./basics/ -I ./geometry/ $(FLAGS)
tmp/geometry_mass.o: ./geometry/mass.cpp tmp/i ./basics/utils.h ./basics/geometry.h ./geometry/polygon.h
	g++ -c ./geometry/mass.cpp -o tmp/geometry_mass.o -I ./basics/ -I ./geometry/ $(FLAGS)
tmp/geometry_hull.o: ./geometry/hull.cpp tmp/i ./basics/utils.h ./basics/geometry.h ./basics/primitives.h ./geometry/polygon.h
	g++ -c ./geometry/hull.cpp -o tmp/geometry_hull.o -I ./basics/ -I ./geometry/ $(FLAGS)
tmp/main.o: ./main.cpp tmp/i ./demos/demos.h
	g++ -c ./main.cpp -o tmp/main.o -I ./demos/ $(FLAGS)
tmp/i:
	mkdir tmp/ -p 
	mkdir build/ -p
	touch tmp/i
