#!/bin/bash

for l in 12
do
    echo "---------------------------l=${l}-------------------------"
    for ((n=1;n<=5;n++))
    do
        /usr/local/Cellar/python/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/Github/MRTA_LTL_same_robot/stap.py
    done
    # for ((n=3;n<4;n++))
done
#     echo "---------------------------n=${n}-------------------------"
#     for h in 10 15 20
#     do
#         echo "---------------------h=${h}-----------------------------"
#         /usr/local/Cellar/python3/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/GitHub/RRT*_LTL/SMT4MulR_2.py ${h}
#     done
# done
