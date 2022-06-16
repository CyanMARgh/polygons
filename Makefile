FLAGS=-lsfml-window -lsfml-system -lsfml-graphics -lpthread -std=c++17 -lm -O3
all: app

app: main.o utils.o polygon.o
	g++ main.o utils.o polygon.o -o app $(FLAGS)

main.o: main.cpp polygon.h
	g++ -c main.cpp -o main.o $(FLAGS)

utils.o: utils.cpp utils.h
	g++ -c utils.cpp -o utils.o $(FLAGS)

polygon.o: polygon.cpp polygon.h utils.h
	g++ -c polygon.cpp -o polygon.o $(FLAGS)
