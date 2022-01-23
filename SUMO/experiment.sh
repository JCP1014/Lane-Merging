#!/bin/sh

# FCFS & DP & Group_DP
########## Density ##########
INDEX=1
while [ $INDEX -le 10 ]
do
    python3 runner_fcfs.py 0.4 100 1 3 F input/input_100_04/${INDEX}.sumocfg --nogui >> output/output_100_04_1.txt
    python3 runner_dp_v2.py 0.4 100 1 3 100 F input/input_100_04/${INDEX}.sumocfg --nogui >> output/output_100_04_1.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.4 100 1 3 F input/input_100_04/${INDEX}.sumocfg >> output/output_100_04_1.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.4 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    python3 runner_fcfs.py 0.5 100 1 3 F input/input_100_05/${INDEX}.sumocfg --nogui >> output/output_100_05_1.txt
    python3 runner_dp_v2.py 0.5 100 1 3 100 F input/input_100_05/${INDEX}.sumocfg --nogui >> output/output_100_05_1.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.5 100 1 3 F input/input_100_05/${INDEX}.sumocfg >> output/output_100_05_1.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.5 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    python3 runner_fcfs.py 0.6 100 1 3 F input/input_100_06/${INDEX}.sumocfg --nogui >> output/output_100_06_1.txt
    python3 runner_dp_v2.py 0.6 100 1 3 100 F input/input_100_06/${INDEX}.sumocfg --nogui >> output/output_100_06_1.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.6 100 1 3 F input/input_100_06/${INDEX}.sumocfg >> output/output_100_06_1.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.6 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    python3 runner_fcfs.py 0.7 100 1 3 F input/input_100_07/${INDEX}.sumocfg --nogui >> output/output_100_07_1.txt
    python3 runner_dp_v2.py 0.7 100 1 3 100 F input/input_100_07/${INDEX}.sumocfg --nogui >> output/output_100_07_1.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.7 100 1 3 F input/input_100_07/${INDEX}.sumocfg >> output/output_100_07_1.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.7 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    python3 runner_fcfs.py 0.8 100 1 3 F input/input_100_08/${INDEX}.sumocfg --nogui >> output/output_100_08_1.txt
    python3 runner_dp_v2.py 0.8 100 1 3 100 F input/input_100_08/${INDEX}.sumocfg --nogui >> output/output_100_08_1.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.8 100 1 3 F input/input_100_08/${INDEX}.sumocfg >> output/output_100_08_1.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.8 are finished!"


########## Number ##########
INDEX=1
while [ $INDEX -le 10 ]
do
    python3 runner_fcfs.py 0.6 20 1 3 F input/input_20_06/${INDEX}.sumocfg --nogui >> output/output_20_06_1.txt
    python3 runner_dp_v2.py 0.6 20 1 3 100 F input/input_20_06/${INDEX}.sumocfg --nogui >> output/output_20_06_1.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.6 20 1 3 F input/input_20_06/${INDEX}.sumocfg >> output/output_20_06_1.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 20 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    python3 runner_fcfs.py 0.6 40 1 3 F input/input_40_06/${INDEX}.sumocfg --nogui >> output/output_40_06_1.txt
    python3 runner_dp_v2.py 0.6 40 1 3 100 F input/input_40_06/${INDEX}.sumocfg --nogui >> output/output_40_06_1.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.6 40 1 3 F input/input_40_06/${INDEX}.sumocfg >> output/output_40_06_1.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 40 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    python3 runner_fcfs.py 0.6 60 1 3 F input/input_60_06/${INDEX}.sumocfg --nogui >> output/output_60_06_1.txt
    python3 runner_dp_v2.py 0.6 60 1 3 100 F input/input_60_06/${INDEX}.sumocfg --nogui >> output/output_60_06_1.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.6 60 1 3 F input/input_60_06/${INDEX}.sumocfg >> output/output_60_06_1.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 60 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    python3 runner_fcfs.py 0.6 80 1 3 F input/input_80_06/${INDEX}.sumocfg --nogui >> output/output_80_06_1.txt
    python3 runner_dp_v2.py 0.6 80 1 3 100 F input/input_80_06/${INDEX}.sumocfg --nogui >> output/output_80_06_1.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.6 80 1 3 F input/input_80_06/${INDEX}.sumocfg >> output/output_80_06_1.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 80 are finished!"




# MILP & Group_MILP
########## Density ##########
INDEX=1
while [ $INDEX -le 10 ]
do
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.4 100 1 3 F input/input_100_04/${INDEX}.sumocfg >> output/output_100_04_2.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.4 100 1 3 F input/input_100_04/${INDEX}.sumocfg >> output/output_100_04_2.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.4 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.5 100 1 3 F input/input_100_05/${INDEX}.sumocfg >> output/output_100_05_2.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.5 100 1 3 F input/input_100_05/${INDEX}.sumocfg >> output/output_100_05_2.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.5 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.6 100 1 3 F input/input_100_06/${INDEX}.sumocfg >> output/output_100_06_2.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.6 100 1 3 F input/input_100_06/${INDEX}.sumocfg >> output/output_100_06_2.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.6 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.7 100 1 3 F input/input_100_07/${INDEX}.sumocfg >> output/output_100_07_2.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.7 100 1 3 F input/input_100_07/${INDEX}.sumocfg >> output/output_100_07_2.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.7 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.8 100 1 3 F input/input_100_08/${INDEX}.sumocfg >> output/output_100_08_2.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.8 100 1 3 F input/input_100_08/${INDEX}.sumocfg >> output/output_100_08_2.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.8 are finished!"


########## Number ##########
INDEX=1
while [ $INDEX -le 10 ]
do
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.6 20 1 3 F input/input_20_06/${INDEX}.sumocfg >> output/output_20_06_2.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.6 20 1 3 F input/input_20_06/${INDEX}.sumocfg >> output/output_20_06_2.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 20 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.6 40 1 3 F input/input_40_06/${INDEX}.sumocfg >> output/output_40_06_2.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.6 40 1 3 F input/input_40_06/${INDEX}.sumocfg >> output/output_40_06_2.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 40 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.6 60 1 3 F input/input_60_06/${INDEX}.sumocfg >> output/output_60_06_2.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.6 60 1 3 F input/input_60_06/${INDEX}.sumocfg >> output/output_60_06_2.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 60 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.6 80 1 3 F input/input_80_06/${INDEX}.sumocfg >> output/output_80_06_2.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.6 80 1 3 F input/input_80_06/${INDEX}.sumocfg >> output/output_80_06_2.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 80 are finished!"



