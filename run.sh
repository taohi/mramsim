#!/bin/sh
./DRAMSim -t traces/k6_aoe_02_short.trc -s system.ini -d ini/DDR3_micron_8M_8B_x16_sg15.ini -c 50000 >stats.txt
grep ^[0-9] stats.txt >latency.txt

