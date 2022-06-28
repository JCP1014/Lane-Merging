all: a.out

C11 = -std=c++11
GUROBI_INCLUDE_PATH = -I /Library/gurobi912/mac64/include
GUROBI_LIBRARY_PATH = -L /Library/gurobi912/mac64/lib
GUROBI_LIBRARY = -lgurobi_c++ -lgurobi91

a.out: main.o generate_input.o utility.o fcfs.o dp_2d.o solution.o window_dp.o reduced_dp.o group_dp.o milp.o group_milp.o
	g++ -o a.out main.o generate_input.o utility.o fcfs.o dp_2d.o solution.o window_dp.o reduced_dp.o group_dp.o milp.o group_milp.o $(GUROBI_INCLUDE_PATH) $(GUROBI_LIBRARY_PATH) $(GUROBI_LIBRARY)
main.o: main.cpp
	g++ $(C11) -c main.cpp $(GUROBI_INCLUDE_PATH) $(GUROBI_LIBRARY_PATH) $(GUROBI_LIBRARY)
generate_input.o: generate_input.cpp generate_input.h
	g++ $(C11) -c generate_input.cpp
utility.o: utility.cpp utility.h
	g++ $(C11) -c utility.cpp
fcfs.o: fcfs.cpp fcfs.h
	g++ $(C11) -c fcfs.cpp
dp_2d.o: dp_2d.cpp dp_2d.h
	g++ $(C11) -c dp_2d.cpp
solution.o: solution.cpp solution.h
	g++ $(C11) -c solution.cpp
dp_3d_twoByTwo.o: dp_3d_twoByTwo.cpp dp_3d_twoByTwo.h
	g++ $(C11) -c dp_3d_twoByTwo.cpp
window_dp.o: window_dp.cpp window_dp.h
	g++ $(C11) -c window_dp.cpp
reduced_dp.o: reduced_dp.cpp reduced_dp.h
	g++ $(C11) -c reduced_dp.cpp
group_dp.o: group_dp.cpp group_dp.h
	g++ $(C11) -c group_dp.cpp
milp.o: milp.cpp milp.h
	g++ $(C11) -c milp.cpp $(GUROBI_INCLUDE_PATH) $(GUROBI_LIBRARY_PATH) $(GUROBI_LIBRARY)
group_milp.o: group_milp.cpp group_milp.h
	g++ $(C11) -c group_milp.cpp $(GUROBI_INCLUDE_PATH) $(GUROBI_LIBRARY_PATH) $(GUROBI_LIBRARY)

clean:
	rm -f a.out main.o fcfs.o generate_input.o utility.o dp_2d.o solution.o dp_3d_twoByTwo.o window_dp.o reduced_dp.o group_dp.o milp.o group_milp.o
