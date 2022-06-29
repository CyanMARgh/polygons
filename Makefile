FLAGS=-lsfml-window -lsfml-system -lsfml-graphics -lpthread -std=c++17 -lm -O3

all: build/app

build/app: tmp/main.o tmp/demo0.o tmp/utils.o tmp/polygon.o tmp/demo1.o tmp/demo2.o tmp/transforms.o tmp/tree.o tmp/demo3.o
	g++ tmp/main.o\
		tmp/demo0.o tmp/demo1.o tmp/demo2.o tmp/demo3.o\
		tmp/polygon.o tmp/tree.o tmp/transforms.o tmp/utils.o -o build/app $(FLAGS)

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



tmp/polygon.o: polygon.cpp polygon.h utils.h transforms.h
	g++ -c polygon.cpp -o tmp/polygon.o $(FLAGS)
tmp/tree.o: tree.cpp tree.h utils.h
	g++ -c tree.cpp -o tmp/tree.o $(FLAGS)	
tmp/transforms.o : transforms.cpp transforms.h utils.h
	g++ -c transforms.cpp -o tmp/transforms.o $(FLAGS)	
tmp/utils.o: utils.cpp utils.h
	g++ -c utils.cpp -o tmp/utils.o $(FLAGS)
	
