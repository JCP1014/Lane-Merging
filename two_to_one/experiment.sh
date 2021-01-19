#!/bin/sh

INDEX=1
STEPS=600
P=6
while [ $INDEX -le 1000 ]
do
	python3 dp_runner.py $STEPS $P --nogui
    retval_dp=$?
    if [ $retval_dp -eq 0 ]; then
        echo OK
    else
        echo FAIL
    fi
	python3 fcfg_runner.py --nogui
    retval_fcfg=$?
    if [ $retval_fcfg -eq 0 ]; then
        echo OK
    else
        echo FAIL
    fi
    if [ $retval_dp -eq 0 ] && [ $retval_fcfg -eq 0 ]; then
        python3 count_time.py >> result_${STEPS}_${P}_w2.txt
        echo $INDEX
        (( INDEX++ ))
    fi
done
echo "$STEPS Finish $INDEX times!"


# INDEX=1
# STEPS=600
# P=7
# while [ $INDEX -le 1000 ]
# do
# 	python3 dp_runner.py $STEPS $P --nogui
#     retval_dp=$?
#     if [ $retval_dp -eq 0 ]; then
#         echo OK
#     else
#         echo FAIL
#     fi
# 	python3 fcfg_runner.py --nogui
#     retval_fcfg=$?
#     if [ $retval_fcfg -eq 0 ]; then
#         echo OK
#     else
#         echo FAIL
#     fi
#     if [ $retval_dp -eq 0 ] && [ $retval_fcfg -eq 0 ]; then
#         python3 count_time.py >> result_${STEPS}_${P}_w4.txt
#         echo $INDEX
#         (( INDEX++ ))
#     fi
# done
# echo "$STEPS Finish $INDEX times!"


# INDEX=1
# STEPS=600
# P=8
# while [ $INDEX -le 1000 ]
# do
# 	python3 dp_runner.py $STEPS $P --nogui
#     retval_dp=$?
#     if [ $retval_dp -eq 0 ]; then
#         echo OK
#     else
#         echo FAIL
#     fi
# 	python3 fcfg_runner.py --nogui
#     retval_fcfg=$?
#     if [ $retval_fcfg -eq 0 ]; then
#         echo OK
#     else
#         echo FAIL
#     fi
#     if [ $retval_dp -eq 0 ] && [ $retval_fcfg -eq 0 ]; then
#         python3 count_time.py >> result_${STEPS}_${P}_w4.txt
#         echo $INDEX
#         (( INDEX++ ))
#     fi
# done
# echo "$STEPS Finish $INDEX times!"