#!/bin/sh

INDEX=1
while [ $INDEX -le 1000 ]
do
	python3 three_to_two.py >> result_01.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        # echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "$STEPS Finish $INDEX times!"