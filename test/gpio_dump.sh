#!/bin/sh
GPIOCHIP=660
BASE=$(cat /sys/class/gpio/gpiochip${GPIOCHIP}/base)
NGPIO=$(cat /sys/class/gpio/gpiochip${GPIOCHIP}/ngpio)
max=$(($BASE+$NGPIO))
gpio=$BASE
while [ $gpio -lt $max ] ; do
    	echo $gpio > /sys/class/gpio/export
    	[ -d /sys/class/gpio/gpio${gpio} ] && {
    		echo in > /sys/class/gpio/gpio${gpio}/direction
    		echo "[GPIO${gpio}] value $(cat /sys/class/gpio/gpio${gpio}/value)"
    		echo ${gpio} > /sys/class/gpio/unexport
    	}
    	gpio=$((gpio+1))
done

