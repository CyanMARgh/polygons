FLAGS=-lsfml-window -lsfml-system -lsfml-graphics -lpthread -std=c++17 -lm -O3

all: build/app

build/app:\
		tmp/main.o\
		tmp/demo0.o tmp/demo1.o tmp/demo2.o tmp/demo3.o tmp/demo4.o\
		tmp/plot.o tmp/primitives.o\
		tmp/polygon.o tmp/mass.o tmp/hull.o\
		tmp/tree.o\
		tmp/utils.o tmp/transforms.o
	g++ tmp/main.o\
		tmp/demo0.o tmp/demo1.o tmp/demo2.o tmp/demo3.o tmp/demo4.o\
		tmp/plot.o tmp/primitives.o\
		tmp/polygon.o tmp/mass.o tmp/hull.o\
		tmp/tree.o\
		tmp/utils.o tmp/transforms.o\
		-o build/app $(FLAGS)

tmp/main.o: main.cpp demos.h
	g++ -c main.cpp -o tmp/main.o $(FLAGS)

tmp/demo0.o: demo0.cpp demos.h polygon.h transforms.h
	g++ -c demo0.cpp -o tmp/demo0.o $(FLAGS)
tmp/demo1.o: demo1.cpp demos.h polygon.h transforms.h
	g++ -c demo1.cpp -o tmp/demo1.o $(FLAGS)
tmp/demo2.o: demo2.cpp demos.h tree.h
	g++ -c demo2.cpp -o tmp/demo2.o $(FLAGS)	
tmp/demo3.o: demo3.cpp demos.h polygon.h
	g++ -c demo3.cpp -o tmp/demo3.o $(FLAGS)	
tmp/demo4.o: demo4.cpp demos.h polygon.h primitives.h
	g++ -c demo4.cpp -o tmp/demo4.o $(FLAGS)	

tmp/primitives.o: primitives.cpp primitives.h polygon.h utils.h
	g++ -c primitives.cpp -o tmp/primitives.o $(FLAGS)
tmp/plot.o: plot.cpp plot.h geometry.h primitives.h
	g++ -c plot.cpp -o tmp/plot.o $(FLAGS)
tmp/hull.o: hull.cpp polygon.h utils.h transforms.h primitives.h
	g++ -c hull.cpp -o tmp/hull.o $(FLAGS)
tmp/mass.o: mass.cpp polygon.h utils.h transforms.h
	g++ -c mass.cpp -o tmp/mass.o $(FLAGS)
tmp/polygon.o: polygon.cpp polygon.h utils.h transforms.h primitives.h
	g++ -c polygon.cpp -o tmp/polygon.o $(FLAGS)


tmp/tree.o: tree.cpp tree.h utils.h tmp/i
	g++ -c tree.cpp -o tmp/tree.o $(FLAGS)	
tmp/transforms.o : transforms.cpp transforms.h utils.h
	g++ -c transforms.cpp -o tmp/transforms.o $(FLAGS)	
tmp/utils.o: utils.cpp utils.h
	g++ -c utils.cpp -o tmp/utils.o $(FLAGS)
	
tmp/i:
	mkdir tmp -p
	mkdir build -p
	touch tmp/i