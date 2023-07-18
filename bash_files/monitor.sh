#!/bin/bash
echo "Logging the measurements..."
interval=1
timestamp_file=$(date +"%Y%m%d-%H%M%S")

#Headers for csv
echo "Timestamp,%Idle" >> /home/locobot/Documents/results/exp$1_cpu_$timestamp_file.csv
echo "Timestamp,Used,Available" >> /home/locobot/Documents/results/exp$1_mem_$timestamp_file.csv

while true; do
    timestamp=$(date +"%Y%m%d-%T")
   
    cpu_data=$(mpstat 1 1| awk 'FNR == 4 {print $13}')
    mem_data=$(free -m | awk 'FNR == 2 {print $3 "," $7}')
    
    echo "$timestamp,$cpu_data" >> /home/locobot/Documents/results/exp$1_cpu_$timestamp_file.csv
    echo "$timestamp,$mem_data" >> /home/locobot/Documents/results/exp$1_mem_$timestamp_file.csv

    sleep $interval #every 9s +1s of the mpstat instruction = 10s
done
