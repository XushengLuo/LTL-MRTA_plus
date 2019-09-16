#!/bin/bash

for w in 50
do
    echo "---------------------------w=${w}-------------------------"
    for n in  12
    do
        echo "--------------------n=${n}-----------------------"
        for ((i=1;i<=5;i++))
        do
            /usr/local/Cellar/python/3.6.3/Frameworks/Python.framework/Versions/3.6/bin/python3.6 /Users/chrislaw/Github/LTL_MRTA_plus/stap.py ${w} ${n}
        done
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
