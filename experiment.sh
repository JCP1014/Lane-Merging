#!/bin/sh

# INDEX=1
# STEPS=60
# while [ $INDEX -le 1000 ]
# do
# 	python3 dp_runner.py $STEPS --nogui
#     retval_dp=$?
#     if [ $retval_dp -eq 0 ]; then
#         echo OK
#     else
#         echo FAIL
#     fi
# 	python3 fafg_runner.py --nogui
#     retval_fafg=$?
#     if [ $retval_fafg -eq 0 ]; then
#         echo OK
#     else
#         echo FAIL
#     fi
#     if [ $retval_dp -eq 0 ] && [ $retval_fafg -eq 0 ]; then
#         python3 count_time.py >> result_$STEPS.txt
#         echo $INDEX
#         (( INDEX++ ))
#     fi
# done
# echo "$STEPS Finish $INDEX times!"

# INDEX=1
# STEPS=300
# while [ $INDEX -le 1000 ]
# do
# 	python3 dp_runner.py $STEPS --nogui
#     retval_dp=$?
#     if [ $retval_dp -eq 0 ]; then
#         echo OK
#     else
#         echo FAIL
#     fi
# 	python3 fafg_runner.py --nogui
#     retval_fafg=$?
#     if [ $retval_fafg -eq 0 ]; then
#         echo OK
#     else
#         echo FAIL
#     fi
#     if [ $retval_dp -eq 0 ] && [ $retval_fafg -eq 0 ]; then
#         python3 count_time.py >> result_$STEPS.txt
#         echo $INDEX
#         (( INDEX++ ))
#     fi
# done
# echo "$STEPS Finish $INDEX times!"


# INDEX=1
# STEPS=600
# while [ $INDEX -le 1000 ]
# do
# 	python3 dp_runner.py $STEPS --nogui
#     retval_dp=$?
#     if [ $retval_dp -eq 0 ]; then
#         echo OK
#     else
#         echo FAIL
#     fi
# 	python3 fafg_runner.py --nogui
#     retval_fafg=$?
#     if [ $retval_fafg -eq 0 ]; then
#         echo OK
#     else
#         echo FAIL
#     fi
#     if [ $retval_dp -eq 0 ] && [ $retval_fafg -eq 0 ]; then
#         python3 count_time.py >> result_$STEPS.txt
#         echo $INDEX
#         (( INDEX++ ))
#     fi
# done
# echo "$STEPS Finish $INDEX times!"


# INDEX=1
# STEPS=1800
# while [ $INDEX -le 1000 ]
# do
# 	python3 dp_runner.py $STEPS --nogui
#     retval_dp=$?
#     if [ $retval_dp -eq 0 ]; then
#         echo OK
#     else
#         echo FAIL
#     fi
# 	python3 fafg_runner.py --nogui
#     retval_fafg=$?
#     if [ $retval_fafg -eq 0 ]; then
#         echo OK
#     else
#         echo FAIL
#     fi
#     if [ $retval_dp -eq 0 ] && [ $retval_fafg -eq 0 ]; then
#         python3 count_time.py >> result_$STEPS.txt
#         echo $INDEX
#         (( INDEX++ ))
#     fi
# done
# echo "$STEPS Finish $INDEX times!"


INDEX=721
STEPS=3600
while [ $INDEX -le 1000 ]
do
	python3 dp_runner.py $STEPS --nogui
    retval_dp=$?
    if [ $retval_dp -eq 0 ]; then
        echo OK
    else
        echo FAIL
    fi
	python3 fafg_runner.py --nogui
    retval_fafg=$?
    if [ $retval_fafg -eq 0 ]; then
        echo OK
    else
        echo FAIL
    fi
    if [ $retval_dp -eq 0 ] && [ $retval_fafg -eq 0 ]; then
        python3 count_time.py >> result_$STEPS.txt
        echo $INDEX
        (( INDEX++ ))
    fi
done
echo "$STEPS Finish $INDEX times!"


INDEX=1
STEPS=10000
while [ $INDEX -le 1000 ]
do
	python3 dp_runner.py $STEPS --nogui
    retval_dp=$?
    if [ $retval_dp -eq 0 ]; then
        echo OK
    else
        echo FAIL
    fi
	python3 fafg_runner.py --nogui
    retval_fafg=$?
    if [ $retval_fafg -eq 0 ]; then
        echo OK
    else
        echo FAIL
    fi
    if [ $retval_dp -eq 0 ] && [ $retval_fafg -eq 0 ]; then
        python3 count_time.py >> result_$STEPS.txt
        echo $INDEX
        (( INDEX++ ))
    fi
done
echo "$STEPS Finish $INDEX times!"