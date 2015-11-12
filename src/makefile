all: main.o POI.o
	g++ -o main main.o POI.o & ./main

main.o: main.cpp POI.h
	g++ -o main.o -c main.cpp

POI.o: POI.cpp POI.h
	g++ -o POI.o -c POI.cpp

clean:
	rm -f *.o main
