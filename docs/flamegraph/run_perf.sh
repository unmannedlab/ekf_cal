../../../../build/ekf_cal/sim perf.yaml perf/ & 
perf record -F 99 -p $! --call-graph dwarf -- sleep 60
perf script > sim.perf
./stackcollapse-perf.pl sim.perf > sim.folded 
./flamegraph.pl sim.folded > flamegraph.svg