all: a.out

CFLAGS = -std=c++11

a.out: main.o generate_input.o fcfs.o dp_2d.o solution.o get_window.o window_dp.o group_dp.o
	g++ -o a.out main.o fcfs.o generate_input.o dp_2d.o solution.o get_window.o window_dp.o group_dp.o
main.o: main.cpp
	g++ $(CFLAGS) -c main.cpp
generate_input.o: generate_input.cpp generate_input.h
	g++ $(CFLAGS) -c generate_input.cpp
fcfs.o: fcfs.cpp fcfs.h
	g++ $(CFLAGS) -c fcfs.cpp
dp_2d.o: dp_2d.cpp dp_2d.h
	g++ $(CFLAGS) -c dp_2d.cpp
solution.o: solution.cpp solution.h
	g++ $(CFLAGS) -c solution.cpp
get_window.o: get_window.cpp get_window.h
	g++ $(CFLAGS) -c get_window.cpp
dp_3d_twoByTwo.o: dp_3d_twoByTwo.cpp dp_3d_twoByTwo.h
	g++ $(CFLAGS) -c dp_3d_twoByTwo.cpp
window_dp.o: window_dp.cpp window_dp.h
	g++ $(CFLAGS) -c window_dp.cpp
group_dp.o: group_dp.cpp group_dp.h
	g++ $(CFLAGS) -c group_dp.cpp

clean:
	rm -f a.out main.o fcfs.o generate_input.o dp_2d.o solution.o get_window.o dp_3d_twoByTwo.o window_dp.o group_dp.o
