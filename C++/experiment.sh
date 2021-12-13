#!/bin/sh

# ########## Density ##########
# INDEX=6
# while [ $INDEX -le 10 ]
# do
#     ./a.out 0.4 100 1 3 input/input_100_04/${INDEX}.dat >> result_density_2.csv
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "Density = 0.4 are finished!"

# INDEX=6
# while [ $INDEX -le 10 ]
# do
#     ./a.out 0.5 100 1 3 input/input_100_05/${INDEX}.dat >> result_density_2.csv
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "Density = 0.5 are finished!"

# INDEX=6
# while [ $INDEX -le 10 ]
# do
#     ./a.out 0.6 100 1 3 input/input_100_06/${INDEX}.dat >> result_density_2.csv
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "Density = 0.6 are finished!"

# INDEX=6
# while [ $INDEX -le 10 ]
# do
#     ./a.out 0.7 100 1 3 input/input_100_07/${INDEX}.dat >> result_density_2.csv
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "Density = 0.7 are finished!"

# INDEX=6
# while [ $INDEX -le 10 ]
# do
#     ./a.out 0.8 100 1 3 input/input_100_08/${INDEX}.dat >> result_density_2.csv
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "Density = 0.8 are finished!"


# ########## Difference between W= and W+ ##########
# INDEX=6
# while [ $INDEX -le 10 ]
# do
#     ./a.out 0.6 100 1 1 input/input_100_06/${INDEX}.dat >> result_gap_2.csv
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "W+ = 1.0 are finished!"

# INDEX=6
# while [ $INDEX -le 10 ]
# do
#     ./a.out 0.6 100 1 1.5 input/input_100_06/${INDEX}.dat >> result_gap_2.csv
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "W+ = 1.5 are finished!"

# INDEX=6
# while [ $INDEX -le 10 ]
# do
#     ./a.out 0.6 100 1 2 input/input_100_06/${INDEX}.dat >> result_gap_2.csv
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "W+ = 2.0 are finished!"

# INDEX=6
# while [ $INDEX -le 10 ]
# do
#     ./a.out 0.6 100 1 2.5 input/input_100_06/${INDEX}.dat >> result_gap_2.csv
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "W+ = 2.5 are finished!"

# INDEX=6
# while [ $INDEX -le 10 ]
# do
#     ./a.out 0.6 100 1 3 input/input_100_06/${INDEX}.dat >> result_gap_2.csv
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "W+ = 3.0 are finished!"


# ######### Number of Vehicles ##########
# INDEX=6
# while [ $INDEX -le 10 ]
# do
#     ./a.out 0.6 20 1 3 input/input_20_06/${INDEX}.dat >> result_number_2.csv
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "Number = 20 are finished!"

# INDEX=6
# while [ $INDEX -le 10 ]
# do
#     ./a.out 0.6 40 1 3 input/input_40_06/${INDEX}.dat >> result_number_2.csv
#     echo $INDEX
#     (( INDEX++ ))
# done
# echo "Number = 40 are finished!"

INDEX=9
while [ $INDEX -le 10 ]
do
    ./a.out 0.6 60 1 3 input/input_60_06/${INDEX}.dat >> result_number_2.csv
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 60 are finished!"

INDEX=6
while [ $INDEX -le 10 ]
do
    ./a.out 0.6 80 1 3 input/input_80_06/${INDEX}.dat >> result_number_2.csv
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 80 are finished!"

INDEX=6
while [ $INDEX -le 10 ]
do
    ./a.out 0.6 100 1 3 input/input_100_06/${INDEX}.dat >> result_number_2.csv
    echo $INDEX
    (( INDEX++ ))
done
echo "Number = 100 are finished!"