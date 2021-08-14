all: a.out

CFLAGS = -std=c++11

a.out: window_based.o generate_input.o fcfs.o dp_2d.o
	g++ -o a.out window_based.o fcfs.o generate_input.o dp_2d.o
window_based.o: window_based.cpp fcfs.h generate_input.h
	g++ $(CFLAGS) -c window_based.cpp
generate_input.o: generate_input.cpp generate_input.h
	g++ $(CFLAGS) -c generate_input.cpp
fcfs.o: fcfs.cpp fcfs.h
	g++ $(CFLAGS) -c fcfs.cpp
dp_2d.o: dp_2d.cpp dp_2d.h
	g++ $(CFLAGS) -c dp_2d.cpp

clean:
	rm -f dp_2d.o fcfs.o generate_input.o window_based.o a.out
