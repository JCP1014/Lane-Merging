#!/bin/sh


######### Number ##########
INDEX=1
while [ $INDEX -le 10 ]
do
    # python3 runner_fcfs.py 0.6 20 1 3 F input/input_20_06/${INDEX}.sumocfg --nogui >> output/output_20_06.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.6 20 1 3 F input/input_20_06/${INDEX}.sumocfg >> output/output_20_06.txt
    # python3 runner_dp_v2.py 0.6 20 1 3 100 F input/input_20_06/${INDEX}.sumocfg --nogui >> output/output_20_06.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.6 20 1 3 F input/input_20_06/${INDEX}.sumocfg >> output/output_20_06.txt
    # LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.6 20 1 3 F input/input_20_06/${INDEX}.sumocfg >> output/output_20_06.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 20 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    # python3 runner_fcfs.py 0.6 40 1 3 F input/input_40_06/${INDEX}.sumocfg --nogui >> output/output_40_06.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.6 40 1 3 F input/input_40_06/${INDEX}.sumocfg >> output/output_40_06.txt
    # python3 runner_dp_v2.py 0.6 40 1 3 100 F input/input_40_06/${INDEX}.sumocfg --nogui >> output/output_40_06.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.6 40 1 3 F input/input_40_06/${INDEX}.sumocfg >> output/output_40_06.txt
    # LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.6 40 1 3 F input/input_40_06/${INDEX}.sumocfg >> output/output_40_06.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 40 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    # python3 runner_fcfs.py 0.6 60 1 3 F input/input_60_06/${INDEX}.sumocfg --nogui >> output/output_60_06.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.6 60 1 3 F input/input_60_06/${INDEX}.sumocfg >> output/output_60_06.txt
    # python3 runner_dp_v2.py 0.6 60 1 3 100 F input/input_60_06/${INDEX}.sumocfg --nogui >> output/output_60_06.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.6 60 1 3 F input/input_60_06/${INDEX}.sumocfg >> output/output_60_06.txt
    # LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.6 60 1 3 F input/input_60_06/${INDEX}.sumocfg >> output/output_60_06.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 60 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    # python3 runner_fcfs.py 0.6 80 1 3 F input/input_80_06/${INDEX}.sumocfg --nogui >> output/output_80_06.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.6 80 1 3 F input/input_80_06/${INDEX}.sumocfg >> output/output_80_06.txt
    # python3 runner_dp_v2.py 0.6 80 1 3 100 F input/input_80_06/${INDEX}.sumocfg --nogui >> output/output_80_06.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.6 80 1 3 F input/input_80_06/${INDEX}.sumocfg >> output/output_80_06.txt
    # LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.6 80 1 3 F input/input_80_06/${INDEX}.sumocfg >> output/output_80_06.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 80 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    # python3 runner_fcfs.py 0.6 100 1 3 F input/input_100_06/${INDEX}.sumocfg --nogui >> output/output_100_06.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.6 100 1 3 F input/input_100_06/${INDEX}.sumocfg >> output/output_100_06.txt
    # python3 runner_dp_v2.py 0.6 100 1 3 100 F input/input_100_06/${INDEX}.sumocfg --nogui >> output/output_100_06.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.6 100 1 3 F input/input_100_06/${INDEX}.sumocfg >> output/output_100_06.txt
    # LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.6 100 1 3 F input/input_100_06/${INDEX}.sumocfg >> output/output_100_06.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 100 are finished!"


INDEX=1
while [ $INDEX -le 10 ]
do
    # python3 runner_fcfs.py 0.4 100 1 3 F input/input_100_04/${INDEX}.sumocfg --nogui >> output/output_100_04.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.4 100 1 3 F input/input_100_04/${INDEX}.sumocfg >> output/output_100_04.txt
    # python3 runner_dp_v2.py 0.4 100 1 3 100 F input/input_100_04/${INDEX}.sumocfg --nogui >> output/output_100_04.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.4 100 1 3 F input/input_100_04/${INDEX}.sumocfg >> output/output_100_04.txt
    # LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.4 100 1 3 F input/input_100_04/${INDEX}.sumocfg >> output/output_100_04.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.4 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    # python3 runner_fcfs.py 0.5 100 1 3 F input/input_100_05/${INDEX}.sumocfg --nogui >> output/output_100_05.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.5 100 1 3 F input/input_100_05/${INDEX}.sumocfg >> output/output_100_05.txt
    # python3 runner_dp_v2.py 0.5 100 1 3 100 F input/input_100_05/${INDEX}.sumocfg --nogui >> output/output_100_05.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.5 100 1 3 F input/input_100_05/${INDEX}.sumocfg >> output/output_100_05.txt
    # LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.5 100 1 3 F input/input_100_05/${INDEX}.sumocfg >> output/output_100_05.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.5 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    # python3 runner_fcfs.py 0.7 100 1 3 F input/input_100_07/${INDEX}.sumocfg --nogui >> output/output_100_07.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.7 100 1 3 F input/input_100_07/${INDEX}.sumocfg >> output/output_100_07.txt
    # python3 runner_dp_v2.py 0.7 100 1 3 100 F input/input_100_07/${INDEX}.sumocfg --nogui >> output/output_100_07.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.7 100 1 3 F input/input_100_07/${INDEX}.sumocfg >> output/output_100_07.txt
    # LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.7 100 1 3 F input/input_100_07/${INDEX}.sumocfg >> output/output_100_07.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.7 are finished!"

INDEX=1
while [ $INDEX -le 10 ]
do
    # python3 runner_fcfs.py 0.8 100 1 3 F input/input_100_08/${INDEX}.sumocfg --nogui >> output/output_100_08.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.8 100 1 3 F input/input_100_08/${INDEX}.sumocfg >> output/output_100_08.txt
    # python3 runner_dp_v2.py 0.8 100 1 3 100 F input/input_100_08/${INDEX}.sumocfg --nogui >> output/output_100_08.txt
    LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.8 100 1 3 F input/input_100_08/${INDEX}.sumocfg >> output/output_100_08.txt
    # LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.8 100 1 3 F input/input_100_08/${INDEX}.sumocfg >> output/output_100_08.txt
    echo $INDEX
    (( INDEX++ ))
done
echo "Density = 0.8 are finished!"


######### Waiting Time ##########
# INDEX=1
# while [ $INDEX -le 10 ]
# do
#     python3 runner_fcfs.py 0.6 100 1 1 F input/input_100_06/${INDEX}.sumocfg --nogui >> output/output_100_06_w10.txt
#     LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.6 100 1 1 F input/input_100_06/${INDEX}.sumocfg >> output/output_100_06_w10.txt
#     python3 runner_dp_v2.py 0.6 100 1 1 100 F input/input_100_06/${INDEX}.sumocfg --nogui >> output/output_100_06_w10.txt
#     LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.6 100 1 1 F input/input_100_06/${INDEX}.sumocfg >> output/output_100_06_w10.txt
#     LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.6 100 1 1 F input/input_100_06/${INDEX}.sumocfg >> output/output_100_06_w10.txt
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "W+ = 1 are finished!"

# INDEX=1
# while [ $INDEX -le 10 ]
# do
#     python3 runner_fcfs.py 0.6 100 1 2 F input/input_100_06/${INDEX}.sumocfg --nogui >> output/output_100_06_w20.txt
#     LD_LIBRARY_PATH=$SUMO_HOME/bin ./milp 0.6 100 1 2 F input/input_100_06/${INDEX}.sumocfg >> output/output_100_06_w20.txt
#     python3 runner_dp_v2.py 0.6 100 1 2 100 F input/input_100_06/${INDEX}.sumocfg --nogui >> output/output_100_06_w20.txt
#     LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_milp 0.6 100 1 2 F input/input_100_06/${INDEX}.sumocfg >> output/output_100_06_w20.txt
#     LD_LIBRARY_PATH=$SUMO_HOME/bin ./group_dp 0.6 100 1 2 F input/input_100_06/${INDEX}.sumocfg >> output/output_100_06_w20.txt
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "W+ = 2 are finished!"
