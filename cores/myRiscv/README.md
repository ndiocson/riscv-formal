
riscv-formal proofs for myRisv
================================

### Quickstart guide:

First install Yosys, SymbiYosys, and the solvers. See [here](http://symbiyosys.readthedocs.io/en/latest/quickstart.html#installing)
for instructions.

To run all standards checks:

```
python3 ../../checks/genchecks.py
make -C checks -j$(nproc)
```

To run again a single check which had failed:

```
#A single time
python3 ../../checks/genchecks.py

#Each time
export test=insn_beq_ch0; rm -r checks/$test; make -C checks -j$(nproc) $test/PASS; python3 disasm.py checks/$test/engine_0/trace.vcd
```


To run imem/dmem checks checks : 

```
sby -f imemcheck.sby
sby -f dmemcheck.sby
```

