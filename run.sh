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

sim_configs["disable_refresh"]="false" # disabling conventional refresh
sim_configs["per_bank_refresh"]="false"
ref_mult="0.5f"
sim_configs["refresh_mult"]=${ref_mult}

# Enable SMD-Combined: SMD-VR, SMD-DRP, and SMD-MS
sim_configs["smd"]=on
configs["smd_refresh_period"]=51200000 # // 51,200,000 cycles = 32ms at 1600Mhz (i.e., 3200 data rate)
configs["smd_ref_policy"]=VariableRefresh
configs["smd_variable_refresh_weak_row_percentage"]="0.1"

configs["smd_rh_protection_enabled"]=true
configs["smd_rh_protection_mac"]=1024
configs["smd_rh_neighbor_refresh_pct"]=1
configs["smd_rh_protection_mode"]="Graphene"

configs["smd_ecc_scrubbing_enabled"]=true
configs["smd_num_scrubbing_machines"]=1
configs["smd_timeout_to_ecc_scrubbing_interval_ratio"]=1
configs["smd_scrub_granularity"]=1
configs["smd_single_scrub_latency"]=552
configs["smd_scrubbing_lock_region"]=bank
configs["smd_ecc_scrubbing_period"]=5760000000000 # 5 minute scrubbing period

# start the simulation
cmd="./ramulator ./configs/SMD_configs/DDR4.cfg --mode=cpu -t ${TRACE_DIR}/${workload_name} $(prep_config_params)"

echo "Running: $cmd"
$cmd
