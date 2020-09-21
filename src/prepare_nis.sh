#!/bin/bash

cat /tmp/ukf_debug_car1.txt | grep RADAR-NIS | sed 's/* RADAR-NIS: //' > /tmp/nis_car1_radar.txt
cat /tmp/ukf_debug_car1.txt | grep LIDAR-NIS | sed 's/* LIDAR-NIS: //' > /tmp/nis_car1_lidar.txt

cp /tmp/nis_car1_radar.txt /vm_share
cp /tmp/nis_car1_lidar.txt /vm_share
