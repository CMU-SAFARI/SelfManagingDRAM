# Self-Managing DRAM (SMD)

Source code of the simulator used to evaluate the mechanisms presented in:
>H. Hassan, M. Patel, J. S. Kim, A. G. Yağlıkçı, N. Vijaykumar, N. Mansouri Ghiasi, S. Ghose, O. Mutlu.
>"[**A Case for Self-Managing DRAM Chips: Improving Performance, Efficiency, Reliability, and Security via Autonomous in-DRAM Maintenance Operations**](https://arxiv.org/abs/2207.13358)".
>In _arXiv_, 2022.

The architectural simulator is built on Ramulator, which is described in this earlier work:

Yoongu Kim, Weikun Yang, and Onur Mutlu, "[Ramulator: A Fast and Extensible DRAM Simulator](https://people.inf.ethz.ch/omutlu/pub/ramulator_dram_simulator-ieee-cal15.pdf)". IEEE Computer Architecture Letters (CAL), March 2015. 

Please cite the above works if you make use of the tool provided in this repository.


## Running Simulations

Ramulator is a cycle-accurate memory simulator that support a wide array of
commercial and academic DRAM standards. We have modified the memory controller
of Ramulator to evaluate the performance of the SMD-based DRAM maintenance
mechanisms proposed in our paper. This version also integrated
[DRAMPower](https://github.com/tukl-msd/DRAMPower), a tool used to estimate DRAM
energy consumption.

### To build Ramulator, just run the following command:
        $ make -j

### To start simulation with default configuration parameters, just run:
        $ ./run.sh

Note that the script will run a very quick simulation using a small trace
file. Please refer to the [original Ramulator
repository](https://github.com/CMU-SAFARI/ramulator) for traces collected
from real workloads.

The `run.sh` script simulates a single-core workload (i.e., "403.gcc" from the
SPEC2006 benchmark suite) using the default system and SMD configuration
parameters. Update the script to simulate a different workload with different
configuration parameters. See `src/Config.h` for a list of the available
configuration parameters.

Also, please refer to the [original Ramulator
repository](https://github.com/CMU-SAFARI/ramulator) for information about the
simulation output and additional details on Ramulator's source code.