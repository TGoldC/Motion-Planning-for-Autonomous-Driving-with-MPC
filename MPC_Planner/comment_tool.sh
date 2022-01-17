#!/bin/bash
#a Tool for commenting specific lines
echo "Type same number if only want to comment one row"
echo "If you use CasADi optimizer and do not install forcespro, please comment line 3-4 and line 55-365 about forcespro"
read -p "start number of comments:" num1
read -p "end number of comments:" num2
read -p "start number of comments (second round):" num3
read -p "end number of comments (second round):" num4
if [ $num2 -ge $num1 -a $num4 -ge $num3 ] >/dev/null 2>&1;then
        echo "Start Comments"
else
        echo -e "\e[TypeError: integer argument expected \e[0m";exit
fi
file=/home/zehua/commonroad/mpfav-ws21-mpc-planner/MPC_Planner/optimizer.py
if [ "$num1" ] && [ "$num2" ];then
        sed -i "${num1},${num2}s/^/#/" $file
 	sed -i "${num3},${num4}s/^/#/" $file
        echo -e "\e[33mok\e[0m"
else 
        echo -e "\e[31mno\e[0m"
fi
