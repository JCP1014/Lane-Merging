all: a.out

CFLAGS = -std=c++11
GUROBI_INCLUDE = -I /Library/gurobi912/mac64/include
GUROBI_LIBRARY = -L /Library/gurobi912/mac64/lib
GUROBI_FLAGS = -lgurobi_c++ -lgurobi91

a.out: main.o generate_input.o fcfs.o dp_2d.o solution.o get_window.o window_dp.o reduced_dp.o get_group.o group_dp.o milp.o group_milp.o
	g++ -o a.out main.o fcfs.o generate_input.o dp_2d.o solution.o get_window.o window_dp.o reduced_dp.o get_group.o group_dp.o milp.o group_milp.o $(GUROBI_INCLUDE) $(GUROBI_LIBRARY) $(GUROBI_FLAGS)
main.o: main.cpp
	g++ $(CFLAGS) -c main.cpp $(GUROBI_INCLUDE) $(GUROBI_LIBRARY) $(GUROBI_FLAGS)
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
reduced_dp.o: reduced_dp.cpp reduced_dp.h
	g++ $(CFLAGS) -c reduced_dp.cpp
get_group.o: get_group.cpp get_group.h
	g++ $(CFLAGS) -c get_group.cpp
group_dp.o: group_dp.cpp group_dp.h
	g++ $(CFLAGS) -c group_dp.cpp
milp.o: milp.cpp milp.h
	g++ $(CFLAGS) -c milp.cpp $(GUROBI_INCLUDE) $(GUROBI_LIBRARY) $(GUROBI_FLAGS)
group_milp.o: group_milp.cpp group_milp.h
	g++ $(CFLAGS) -c group_milp.cpp $(GUROBI_INCLUDE) $(GUROBI_LIBRARY) $(GUROBI_FLAGS)

clean:
	rm -f a.out main.o fcfs.o generate_input.o dp_2d.o solution.o get_window.o dp_3d_twoByTwo.o window_dp.o reduced_dp.o get_group.o group_dp.o milp.o group_milp.o
