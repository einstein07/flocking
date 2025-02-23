#!/bin/bash
# run_experiments.sh
# Runs ARGoS in background and ROS2 in foreground with logs organized by population size

# Configuration
ROBOT_COUNTS=(10 20 40 80 160 320 640 1280 2560 5120 10240)
REPETITIONS=10
RESULTS_FILE="/mnt/scalability_results_$(date +%Y%m%d_%H%M%S).csv"
CONFIG_DIR="/opt/ros2_ws/src/flocking/launch/20_threads"
LAUNCH_FILE="/tmp/argos_interface.launch.py"
ARGOS_EXEC="argos3"
ROS2_TIME_FILE="/tmp/ros2_time_$$.tmp"
TIMEOUT_SECONDS=20
SHUTDOWN_GRACE_SECONDS=5
SAMPLE_INTERVAL=1

# Environment setup
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3:/opt/ros2_ws/install/argos3_ros2_bridge/lib
export ARGOS_PLUGIN_PATH=/opt/ros2_ws/install/argos3_ros2_bridge/lib/
#export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 workspace setup
source /opt/ros2_ws/install/setup.bash
echo $RMW_IMPLEMENTATION
ulimit -n 999999
ulimit -n

# Initialize results file
echo "Robots,Repetition,WallTime(s),CPU(%),MaxMem(kB),ArgosCPU(%),Ros2CPU(%),ArgosMem(kB),Ros2Mem(kB)" > "$RESULTS_FILE"

# Cleanup function
cleanup() {
    echo "Cleaning up..." >&2
    pkill -f "argos3" 2>/dev/null
    pkill -f "ros2|flocking" 2>/dev/null
    rm -f "$LAUNCH_FILE" "time_output_$$_$ROBOTS_$rep.tmp" "$ROS2_TIME_FILE"
    # Only remove temporary files, keep logs in directories
    sleep 2
    if pgrep -f "ros2|flocking" > /dev/null; then
        echo "Stray processes detected, forcing kill..." >&2
        pkill -9 -f "ros2|flocking" 2>/dev/null
    fi
}

# Trap signals
#trap cleanup INT TERM EXIT

for ROBOTS in "${ROBOT_COUNTS[@]}"; do
    CONFIG_FILE="${CONFIG_DIR}/flocking${ROBOTS}.argos"
    if [[ ! -f "$CONFIG_FILE" ]]; then
        echo "Config file $CONFIG_FILE not found. Skipping robot count $ROBOTS." >&2
        continue
    fi
    echo "Running experiments for $ROBOTS robots..." >&2
    
    # Create directory for this population size
    LOG_DIR="/mnt/20_threads/${ROBOTS}-robots"
    mkdir -p "$LOG_DIR"
    
    
    for (( rep=1; rep<=REPETITIONS; rep++ )); do
        echo "  Repetition $rep for $ROBOTS robots..." >&2
         # Clean up before each robot count
    	#cleanup
        # Generate launch file
        cat > "$LAUNCH_FILE" << EOL
import os
import pathlib
import launch
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join('/opt/ros2_ws/src/flocking', 'config')
    param_config = os.path.join(config_dir, "config.yaml")
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)["flocking"]["ros__parameters"]
    
    ld = LaunchDescription()
EOL
        for ((i=0; i<ROBOTS; i++)); do
            namespace="bot$i"
            domain_id=$((i / 115))  # Group by 115: 0-114 = 0, 115-229 = 1, etc.
            echo "    ${namespace} = Node(" >> "$LAUNCH_FILE"
            echo "        package=\"argos3_ros2_bridge\"," >> "$LAUNCH_FILE"
            echo "        executable=\"flocking\"," >> "$LAUNCH_FILE"
            echo "        name=\"flocking\"," >> "$LAUNCH_FILE"
            echo "        output=\"screen\"," >> "$LAUNCH_FILE"
            echo "        namespace=\"${namespace}\"," >> "$LAUNCH_FILE"
            echo "        additional_env={\"ROS_DOMAIN_ID\": \"$domain_id\"}," >> "$LAUNCH_FILE"
            echo "        parameters=[params]" >> "$LAUNCH_FILE"
            echo "    )" >> "$LAUNCH_FILE"
            echo "    ld.add_action(${namespace})" >> "$LAUNCH_FILE"
        done
        echo "    return ld" >> "$LAUNCH_FILE"

        # Start timing
        start_time=$(date +%s.%N)

        # Launch Argos in background
        TEMP_TIME_FILE="time_output_$$_$ROBOTS_$rep.tmp"
        /usr/bin/time -f "%e %P %M" -o "$TEMP_TIME_FILE" $ARGOS_EXEC -c "$CONFIG_FILE" -z &
        ARGOS_PID=$!

        # Give ARGoS a moment to start
        sleep 60

        # Launch ROS2 in foreground with output logging to population directory
        ROS2_OUTPUT_FILE="${LOG_DIR}/ros2_output_${ROBOTS}_${rep}.log"
        ( /usr/bin/time -f "%e %P %M" -o "$ROS2_TIME_FILE" ros2 launch "$LAUNCH_FILE" ) > "$ROS2_OUTPUT_FILE" 2>&1 &
        ROS2_PID=$!

        # Monitor ROS2 usage with precise node counting
        ros2_cpu_total=0
        ros2_mem_total=0
        sample_count=0
        num_cores=24
        DEBUG_FILE="${LOG_DIR}/debug_${ROBOTS}_${rep}.log"
        while kill -0 $ARGOS_PID 2>/dev/null; do
            ros2_cpu_sum=0
            ros2_mem_sum=0
            active_nodes=0
            node_list=$(ros2 node list 2>/dev/null | grep "/bot[0-9]" | wc -l)
            for pid in $(pgrep -f "flocking" | grep -v "ros2 launch"); do
                metrics=$(ps -p $pid -o %cpu,rss --no-headers 2>/dev/null)
                if [[ -n "$metrics" && $active_nodes -lt $ROBOTS ]]; then
                    cpu=$(echo "$metrics" | awk '{print $1}')
                    mem=$(echo "$metrics" | awk '{print $2}')
                    ros2_cpu_sum=$(echo "$ros2_cpu_sum + $cpu" | bc -l)
                    ros2_mem_sum=$(echo "$ros2_mem_sum + $mem" | bc -l)
                    ((active_nodes++))
                fi
            done
            if [[ $active_nodes -gt $ROBOTS ]]; then
                active_nodes=$ROBOTS
            fi
            ros2_cpu_avg=$(echo "scale=2; $ros2_cpu_sum / $num_cores" | bc -l)
            ros2_cpu_total=$(echo "$ros2_cpu_total + $ros2_cpu_avg" | bc -l)
            [[ $(echo "$ros2_mem_sum > $ros2_mem_total" | bc -l) -eq 1 ]] && ros2_mem_total=$ros2_mem_sum
            ((sample_count++))
            echo "Sample $sample_count: $active_nodes nodes (ROS2 list: $node_list), CPU=$ros2_cpu_avg%, Mem=$ros2_mem_sum kB" >> "$DEBUG_FILE"
            sleep $SAMPLE_INTERVAL
        done

        # Wait for Argos to finish
        wait $ARGOS_PID 2>/dev/null
        echo "ARGoS finished at $(date)" >> "$DEBUG_FILE"

        # Attempt clean ROS2 shutdown
        kill -INT $ROS2_PID 2>/dev/null
        echo "Sent SIGINT to ROS2 PID $ROS2_PID at $(date)" >> "$DEBUG_FILE"

        # Wait for ROS2 to shutdown with timeout
        timeout_counter=0
        while kill -0 $ROS2_PID 2>/dev/null; do
            if [[ $timeout_counter -ge $TIMEOUT_SECONDS ]]; then
                echo "ROS2 launch did not shut down cleanly after $TIMEOUT_SECONDS seconds, escalating to SIGTERM..." >&2
                kill -TERM $ROS2_PID 2>/dev/null
                sleep $SHUTDOWN_GRACE_SECONDS
                if kill -0 $ROS2_PID 2>/dev/null; then
                    echo "ROS2 still running, forcing termination with SIGKILL..." >&2
                    kill -KILL $ROS2_PID 2>/dev/null
                    sleep 1
                    pkill -f "ros2|flocking" 2>/dev/null
                fi
                break
            fi
            sleep 1
            ((timeout_counter++))
        done

        # Read Argos metrics
        if [[ -f "$TEMP_TIME_FILE" ]]; then
            read argos_wall_time argos_cpu argos_mem < "$TEMP_TIME_FILE" 2>/dev/null
            argos_cpu=$(echo "$argos_cpu" | tr -d '%')
            echo "Argos metrics: $argos_wall_time $argos_cpu $argos_mem" >> "$DEBUG_FILE"
            rm -f "$TEMP_TIME_FILE"
        else
            echo "Argos time file missing" >> "$DEBUG_FILE"
            argos_wall_time=0
            argos_cpu=0
            argos_mem=0
        fi

        # Read ROS2 metrics from time file if available
        if [[ -f "$ROS2_TIME_FILE" && -s "$ROS2_TIME_FILE" ]]; then
            read ros2_wall_time ros2_cpu ros2_mem < "$ROS2_TIME_FILE" 2>/dev/null
            ros2_cpu=$(echo "$ros2_cpu" | tr -d '%')
            echo "ROS2 metrics from time: $ros2_wall_time $ros2_cpu $ros2_mem" >> "$DEBUG_FILE"
            rm -f "$ROS2_TIME_FILE"
        else
            if [[ $sample_count -gt 0 ]]; then
                ros2_cpu=$(echo "scale=2; $ros2_cpu_total / $sample_count" | bc -l)
                ros2_mem=$ros2_mem_total
            else
                ros2_cpu=0
                ros2_mem=0
            fi
            echo "ROS2 metrics from monitoring: CPU=$ros2_cpu%, Mem=$ros2_mem kB" >> "$DEBUG_FILE"
        fi

        # Determine wall time
        if [[ -n "$argos_wall_time" && -n "$ros2_wall_time" && $(echo "$argos_wall_time > 0" | bc -l) -eq 1 && $(echo "$ros2_wall_time > 0" | bc -l) -eq 1 ]]; then
            if [[ $(echo "$argos_wall_time > $ros2_wall_time" | bc -l) -eq 1 ]]; then
                wall_time=$argos_wall_time
            else
                wall_time=$ros2_wall_time
            fi
        else
            wall_time=$(echo "scale=2; $(date +%s.%N) - $start_time" | bc -l)
        fi

        # Total CPU and memory
        total_cpu=$(echo "scale=2; (${argos_cpu:-0} + ${ros2_cpu:-0})" | bc -l 2>/dev/null || echo 0)
        total_mem=$(echo "${argos_mem:-0} + ${ros2_mem:-0}" | bc -l 2>/dev/null || echo 0)

        # Log detailed results
        echo "$ROBOTS,$rep,$wall_time,$total_cpu,$total_mem,${argos_cpu:-0},${ros2_cpu:-0},${argos_mem:-0},${ros2_mem:-0}" >> "$RESULTS_FILE"
    done
done

echo "All experiments complete. Results saved to $RESULTS_FILE." >&2
#cleanup
