all: sumo_runner

C11 = -std=c++11
GUROBI_INCLUDE_PATH = -I /Library/gurobi912/mac64/include
GUROBI_LIBRARY_PATH = -L /Library/gurobi912/mac64/lib
GUROBI_LIBRARY = -lgurobi_c++ -lgurobi91
SUMO_INCLUDE_PATH = -I$$SUMO_HOME/src
SUMO_LIBRARY_PATH = -L$$SUMO_HOME/bin
SUMO_LIBRARY = -ltracicpp

sumo_runner: main.o generate_input.o utility.o get_vehicle_info.o solution.o fafg.o milp.o dp.o group_milp.o group_dp.o control.o 
	g++ -o sumo_runner main.o generate_input.o utility.o get_vehicle_info.o solution.o fafg.o milp.o dp.o group_milp.o group_dp.o control.o $(GUROBI_INCLUDE_PATH) $(GUROBI_LIBRARY_PATH) $(GUROBI_LIBRARY) $(SUMO_INCLUDE_PATH) $(SUMO_LIBRARY_PATH) $(SUMO_LIBRARY)
main.o: main.cpp
	g++ -c $(C11) main.cpp $(GUROBI_INCLUDE_PATH) $(GUROBI_LIBRARY_PATH) $(GUROBI_LIBRARY) $(SUMO_INCLUDE_PATH) $(SUMO_LIBRARY_PATH) $(SUMO_LIBRARY)
generate_input.o: generate_input.cpp generate_input.h
	g++ -c $(C11) generate_input.cpp
utility.o: utility.cpp utility.h
	g++ -c $(C11) utility.cpp
get_vehicle_info.o: get_vehicle_info.cpp get_vehicle_info.h
	g++ -c $(C11) get_vehicle_info.cpp $(SUMO_INCLUDE_PATH) $(SUMO_LIBRARY_PATH) $(SUMO_LIBRARY)
solution.o: solution.cpp solution.h
	g++ -c $(C11) solution.cpp
fafg.o: fafg.cpp fafg.h
	g++ -c $(C11) fafg.cpp $(SUMO_INCLUDE_PATH) $(SUMO_LIBRARY_PATH) $(SUMO_LIBRARY)
milp.o: milp.cpp milp.h
	g++ -c $(C11) milp.cpp $(GUROBI_INCLUDE_PATH) $(GUROBI_LIBRARY_PATH) $(GUROBI_LIBRARY) $(SUMO_INCLUDE_PATH) $(SUMO_LIBRARY_PATH) $(SUMO_LIBRARY)
dp.o: dp.cpp dp.h
	g++ -c $(C11) dp.cpp $(SUMO_INCLUDE_PATH) $(SUMO_LIBRARY_PATH) $(SUMO_LIBRARY)
group_milp.o: group_milp.cpp group_milp.h
	g++ -c $(C11) group_milp.cpp $(GUROBI_INCLUDE_PATH) $(GUROBI_LIBRARY_PATH) $(GUROBI_LIBRARY) $(SUMO_INCLUDE_PATH) $(SUMO_LIBRARY_PATH) $(SUMO_LIBRARY)
group_dp.o: group_dp.cpp group_dp.h
	g++ -c $(C11) group_dp.cpp $(SUMO_INCLUDE_PATH) $(SUMO_LIBRARY_PATH) $(SUMO_LIBRARY)
control.o: control.cpp control.h
	g++ -c $(C11) control.cpp $(GUROBI_INCLUDE_PATH) $(GUROBI_LIBRARY_PATH) $(GUROBI_LIBRARY) $(SUMO_INCLUDE_PATH) $(SUMO_LIBRARY_PATH) $(SUMO_LIBRARY)


clean:
	rm -f sumo_runner main.o generate_input.o utility.o get_vehicle_info.o solution.o fafg.o milp.o dp.o group_milp.o group_dp.o control.o 
