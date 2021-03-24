#!/bin/sh

INDEX=1
while [ $INDEX -le 10 ]
do
	python3 window_based.py 2.7 67 1 3 >> record_09_67.txt
    retval=$?
    if [ $retval -eq 0 ]; then
        echo $INDEX
        (( INDEX++ ))
    else
        echo FAIL
    fi
done
echo "lambda=0.9 Finish $((--INDEX)) times!"