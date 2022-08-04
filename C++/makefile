all: main

C11 = -std=c++11
GUROBI_INCLUDE_PATH = -I /Library/gurobi912/mac64/include
GUROBI_LIBRARY_PATH = -L /Library/gurobi912/mac64/lib
GUROBI_LIBRARY = -lgurobi_c++ -lgurobi91

main: main.o generate_input.o utility.o fafg.o dp_2d.o solution.o window_dp.o pruned_dp.o group_dp.o milp.o group_milp.o
	g++ -o main main.o generate_input.o utility.o fafg.o dp_2d.o solution.o window_dp.o pruned_dp.o group_dp.o milp.o group_milp.o $(GUROBI_INCLUDE_PATH) $(GUROBI_LIBRARY_PATH) $(GUROBI_LIBRARY)
main.o: main.cpp
	g++ $(C11) -c main.cpp $(GUROBI_INCLUDE_PATH) $(GUROBI_LIBRARY_PATH) $(GUROBI_LIBRARY)
generate_input.o: generate_input.cpp generate_input.h
	g++ $(C11) -c generate_input.cpp
utility.o: utility.cpp utility.h
	g++ $(C11) -c utility.cpp
fafg.o: fafg.cpp fafg.h
	g++ $(C11) -c fafg.cpp
dp_2d.o: dp_2d.cpp dp_2d.h
	g++ $(C11) -c dp_2d.cpp
solution.o: solution.cpp solution.h
	g++ $(C11) -c solution.cpp
window_dp.o: window_dp.cpp window_dp.h
	g++ $(C11) -c window_dp.cpp
pruned_dp.o: pruned_dp.cpp pruned_dp.h
	g++ $(C11) -c pruned_dp.cpp
group_dp.o: group_dp.cpp group_dp.h
	g++ $(C11) -c group_dp.cpp
milp.o: milp.cpp milp.h
	g++ $(C11) -c milp.cpp $(GUROBI_INCLUDE_PATH) $(GUROBI_LIBRARY_PATH) $(GUROBI_LIBRARY)
group_milp.o: group_milp.cpp group_milp.h
	g++ $(C11) -c group_milp.cpp $(GUROBI_INCLUDE_PATH) $(GUROBI_LIBRARY_PATH) $(GUROBI_LIBRARY)

clean:
	rm -f main main.o fafg.o generate_input.o utility.o dp_2d.o solution.o window_dp.o pruned_dp.o group_dp.o milp.o group_milp.o
