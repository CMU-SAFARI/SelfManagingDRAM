# Self-Managing DRAM (SMD)

Source code of the simulator used to evaluate the mechanisms presented in:
>H. Hassan, A. Olgun, A. G. Yağlıkçı, H. Luo, and O. Mutlu.
>"[**A Case for Self-Managing DRAM Chips: Improving Performance, Efficiency, Reliability, and Security via Autonomous in-DRAM Maintenance Operations**](https://arxiv.org/abs/2207.13358)".
>In _arXiv_, 2022.

The architectural simulator is built on Ramulator, which is described in this earlier work:

Yoongu Kim, Weikun Yang, and Onur Mutlu, "[Ramulator: A Fast and Extensible DRAM Simulator](https://people.inf.ethz.ch/omutlu/pub/ramulator_dram_simulator-ieee-cal15.pdf)". IEEE Computer Architecture Letters (CAL), March 2015. 

Please cite the above works if you make use of the tool provided in this repository.

## Dependencies

The simulator integrates [DRAMPower](https://github.com/tukl-msd/DRAMPower) for DRAM power consumption and energy analysis. DRAMPower depends on `libxerces`, which can be installed using `apt install libxerces-c-dev`.

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
SPEC2006 benchmark suite) using the default system and the SMD-combined configuration with the deterministic RowHammer protection mechanism. 
Update the script to simulate a different workload with different
configuration parameters. See `src/Config.h` for a list of the available
configuration parameters.

We provide multiple bash scripts under the `scripts` directory. Each script sets the simulation configuration parameters for one of the SMD configurations evaluated in the paper. You can replace the contents of `run.sh` with the content of a script under `scripts` to simulate a different SMD configuration. We briefly describe the configurations provided in the `scripts` directory:

* SMD-Combined-DRP.sh: SMD combined with the deterministic RowHammer protection mechanism (Graphene).
* SMD-Combined-PRP.sh: SMD combined with the probabilistic RowHammer protection mechanism (PARA).
* SMD-DRP.sh: SMD deterministic RowHammer protection mechanism (Graphene) + SMD fixed rate refresh (SMD-FR).
* SMD-FR.sh: SMD fixed rate refresh with a refresh period of 32 ms.
* SMD-MS.sh: SMD memory scrubbing with a 5 minute scrubbing period + SMD fixed rate refresh (SMD-FR).
* SMD-PRP.sh: SMD probabilistic RowHammer protection mechanism (PARA) + SMD fixed rate refresh (SMD-FR).
* SMD-PRPplus.sh: SMD's area-efficient RowHammer protection mechanism with bloom filters + SMD fixed rate refresh (SMD-FR).
* SMD-VR.sh: SMD variable rate refresh with a refresh period of 32 ms and a weak row probability of 0.1%.

Also, please refer to the [original Ramulator
repository](https://github.com/CMU-SAFARI/ramulator) for information about the
simulation output and additional details on Ramulator's source code.
