#!/bin/sh
./DRAMSim -t traces/mase_art.trc -s system.ini -d ini/DDR3_micron_8M_8B_x16_sg15.ini -c 600000 >stats.txt
grep ^[0-9] stats.txt >latency.txt
grep "^r" stats.txt >banklist.txt

