#!/bin/bash

function prep_config_params {
        base_confs=$(declare -p "configs")
        sim_confs=$(declare -p "sim_configs")
        eval "declare -A base_arr="${base_confs#*=}
        eval "declare -A sim_arr="${sim_confs#*=}

        conf_string=" "
    for conf_key in "${!base_arr[@]}"
    do
        conf_string="${conf_string}-p $conf_key=${base_arr[$conf_key]} "
    done

    for conf_key in "${!sim_arr[@]}"
    do
        conf_string="${conf_string}-c $conf_key=${sim_arr[$conf_key]} "
    done

    echo $conf_string
}

TRACE_DIR="./trace_files/"
workload_name="403.gcc"

mkdir -p ${TRACE_DIR}

if [ ! -f "${TRACE_DIR}/${workload_name}" ]; then
    # the trace file is missing. Download the compressed version of the trace file from the main Ramulator github repository
    echo "Downloading '${workload_name}' from https://github.com/CMU-SAFARI/ramulator"
    wget "https://github.com/CMU-SAFARI/ramulator/raw/master/cputraces/${workload_name}.gz" -P ${TRACE_DIR}

    echo "Decompressing the trace file..."
    gunzip "${TRACE_DIR}/${workload_name}.gz"
fi

# Configuration parameters
# 'configs' specifies parameters used from the start of the simulation (i.e., including warmup)
# 'sim_configs' specifies parameters that are applied right after warmup ends

declare -A configs 
configs["org"]="DDR4_16Gb_x8"
configs["channels"]=4
configs["ranks"]=2
configs["speed"]="DDR4_3200"
configs["subarray_size"]=8192
configs["warmup_insts"]=100000000 # warmup the caches for 100 million instructions
configs["expected_limit_insts"]=100000000 # simulate 100 million instructions after warmup
configs["l3_size"]=4194304 # 4MiB LLC
configs["translation"]="Random"
configs["row_policy"]="timeout"
configs["timeout_row_policy_threshold"]=360


# Configuration parameter updates applied after the warmup
declare -A sim_configs

sim_configs["disable_refresh"]="false" # disable/enable conventional refresh
sim_configs["per_bank_refresh"]="false" # disable/enable per bank refresh
ref_mult="0.5f" # multiply the default refresh interval (64 ms) by this number
sim_configs["refresh_mult"]=${ref_mult}

# Enable SMD-PRP
sim_configs["smd"]=on # enable/disable SMD
configs["smd_refresh_period"]=51200000 # 51,200,000 cycles = 32ms at 1600Mhz (i.e., 3200 data rate)
configs["smd_ref_policy"]=FixedRate # variable rate refresh ("VariableRefresh") or fixed rate refresh ("FixedRate")

configs["smd_rh_protection_enabled"]=true # disable/enable SMD deterministic/probabilistic RowHammer protection
configs["smd_rh_protection_mac"]=1024 # ACTmax threshold for SMD RowHammer protection
configs["smd_rh_neighbor_refresh_pct"]=1 # (0-100) the probability of neighbor row refresh to occur when a row is activated (relevant for probabilistic protection)
configs["smd_rh_protection_mode"]="Graphene" # deterministic ("Graphene") or probabilistic ("PARA") RowHammer protection

configs["smd_ecc_scrubbing_enabled"]=false # enable/disable ECC scrubbing (SMD-MS)
configs["smd_num_scrubbing_machines"]=1 # the number of banks that are scrubbed concurrently
configs["smd_scrubbing_granularity"]=1 # number of rows scrubbed within a single smd region lock
configs["smd_single_scrubbing_latency"]=552 # simulated latency (in number of memory controller cycles) for a scrub operation
configs["smd_scrubbing_lock_region"]=bank # valid entries: 'bank' and 'region'. When bank, scrubbing puts an entire bank under maintenance whereas region operates as in smd refresh
configs["smd_ecc_scrubbing_period"]=5760000000000 # 5 minute scrubbing period, the time it takes for the scrubber to process all DRAM rows

# start the simulation
cmd="./ramulator ./configs/SMD_configs/DDR4.cfg --mode=cpu -t ${TRACE_DIR}/${workload_name} $(prep_config_params)"

echo "Running: $cmd"
$cmd
