#!/usr/bin/python
#encoding:utf-8
f=open("latency.txt",'r')
data = f.readlines()
total_req = 0
total_latency = 0
for line in data:
    l = line.split(':')
    req_counter=int((l[1].split('\n'))[0])
    total_req += req_counter
    latency=int(l[0])
    total_latency += req_counter*latency
#    print latency,':',req_counter,'\n'
print "total_latency:",total_latency,'Cycles\n'
print "total_req:",total_req,'\n'
print "Cycles per req(in average):",total_latency/total_req,'\n'

