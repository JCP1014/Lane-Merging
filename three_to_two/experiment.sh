#!/bin/sh

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.01 100 1 3 >> result_01.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "lambda=0.1 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.02 100 1 3 >> result_02.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "lambda=0.2 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.03 100 1 3 >> result_03.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "lambda=0.3 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 100 1 3 >> result_04.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "lambda=0.4 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.05 100 1 3 >> result_05.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "lambda=0.5 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 20 1 3 >> result_20.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "N=20 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 40 1 3 >> result_40.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "N=40 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 60 1 3 >> result_60.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "N=60 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 80 1 3 >> result_80.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "N=80 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 100 1 3 >> result_100.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "N=100 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 100 1.0 3 >> result_w10.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "W==1 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 100 1.2 3 >> result_w12.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "W==1.2 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 100 1.4 3 >> result_w14.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "W==1.4 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 100 1.6 3 >> result_w16.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "W==1.6 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 100 1.8 3 >> result_w18.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "W==1.8 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 100 2.0 3 >> result_w20.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "W==2.0 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 100 2.2 3 >> result_w22.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "W==2.2 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 100 2.4 3 >> result_w24.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "W==2.4 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 100 2.6 3 >> result_w26.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "W==2.6 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 100 2.8 3 >> result_w28.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "W==2.8 Finish $INDEX times!"

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py 0.04 100 3.0 3 >> result_w30.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "W==3.0 Finish $INDEX times!"