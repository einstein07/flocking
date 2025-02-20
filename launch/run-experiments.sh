#!/bin/bash
# run_experiments.sh
# This script automates running experiments for different robot populations.
# For each population size, it runs 20 repetitions, launching the Argos simulation
# and ROS2 controllers concurrently. It records wall-clock time, CPU usage,
# and maximum memory usage for each run.

# Configuration:
ROBOT_COUNTS=(10 20 40 80 160 320 640 1280 2560 5120 10240)
REPETITIONS=10
RESULTS_FILE="scalability_results.csv"

# Directory where your Argos config files reside.
# Files should be named like: flocking_20.argos, flocking_40.argos, etc.
CONFIG_DIR="."  # Change to your actual directory
# Path to the ROS2 launch file that starts your controllers.
LAUNCH_FILE=/tmp/argos_interface.launch.py # Change accordingly

# Argos executable (modify if necessary)
ARGOS_EXEC="argos3"

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3:/home/sindiso/ros2_dev/install/argos3_ros2_bridge/lib
export ARGOS_PLUGIN_PATH=/home/sindiso/ros2_dev/install/argos3_ros2_bridge/lib/
# keep topics local to computer
export ROS_LOCALHOST_ONLY=1

# Initialize results file with a header.
echo "Robots,Repetition,WallTime(s),CPU,MaxMem(kB)" > "$RESULTS_FILE"

for ROBOTS in "${ROBOT_COUNTS[@]}"; do
    CONFIG_FILE="${CONFIG_DIR}/flocking${ROBOTS}.argos"
    if [[ ! -f "$CONFIG_FILE" ]]; then
        echo "Config file $CONFIG_FILE not found. Skipping robot count $ROBOTS."
        continue
    fi
    echo "Running experiments for $ROBOTS robots..."
    
    for (( rep=1; rep<=REPETITIONS; rep++ )); do
        echo "  Repetition $rep for $ROBOTS robots..."
        
        # Temporary file to capture GNU time output.
        TEMP_TIME_FILE="time_output.tmp"
        rm -f "$TEMP_TIME_FILE"
        
        #############################
        # Launch file for this run
        ############################
	echo "import os" > $LAUNCH_FILE
	echo -e "import pathlib" >> $LAUNCH_FILE
	echo -e "import launch" >> $LAUNCH_FILE
	echo -e "import yaml" >> $LAUNCH_FILE
	echo -e "from launch import LaunchDescription" >> $LAUNCH_FILE
	echo -e "from launch_ros.actions import Node" >> $LAUNCH_FILE

	echo -e "from ament_index_python.packages import get_package_share_directory" >> $LAUNCH_FILE

	echo -e "def generate_launch_description():" >> $LAUNCH_FILE
	echo -e "\tconfig_dir = os.path.join('/home/sindiso/ros2_dev/src/flocking', 'config')" >> $LAUNCH_FILE
	echo -e "\tparam_config = os.path.join(config_dir, \"config.yaml\")" >> $LAUNCH_FILE
	echo -e "\twith open(param_config, 'r') as f:" >> $LAUNCH_FILE
	echo -e "\t\tparams = yaml.safe_load(f)[\"flocking\"][\"ros__parameters\"]" >> $LAUNCH_FILE
	    
	echo -e "\tld = LaunchDescription()" >> $LAUNCH_FILE

	for ((i=0; i<$ROBOTS; i++)); do
	    namespace="bot$i"
	    echo -e "\t$namespace = Node(package=\"argos3_ros2_bridge\", executable=\"flocking\", name=\"flocking\", output=\"screen\", namespace=\"$namespace\", parameters=[params])" >> $LAUNCH_FILE
	    echo -e "\tld.add_action($namespace)" >> $LAUNCH_FILE
	done >> $LAUNCH_FILE
	echo -e "\treturn ld" >> $LAUNCH_FILE
	########################################
	# End of launch file
	#######################################
        
        
        # Launch Argos simulation in background and then the ROS2 launch.
        # Wrap the combined command in /usr/bin/time to capture metrics.
        /usr/bin/time -f "row=%e, %P, %M" -o "$TEMP_TIME_FILE" bash -c "\
            $ARGOS_EXEC -c $CONFIG_FILE -z & \
            argos_pid=\$!; \
            ros2 launch $LAUNCH_FILE; \
            wait \$argos_pid"

        #ros2 launch $LAUNCH_FILE &
        #sleep 1     
    	#/usr/bin/time -f "row=%e, %P, %M" -o "$TEMP_TIME_FILE" bash -c "\
    	#    $ARGOS_EXEC -c $CONFIG_FILE -z
    	#    argos_pid=\$!; \
        #    ros2 launch $LAUNCH_FILE; \
        #    wait \$argos_pid"
        
        EXIT_CODE=$?
        if [[ $EXIT_CODE -ne 0 ]]; then
            echo "Simulation failed for $ROBOTS robots on repetition $rep." >&2
            continue
        fi
        
        # Extract metrics from the temporary file.
        ROW=$(grep "row=" "$TEMP_TIME_FILE" | sed 's/row=//')
        #CPU=$(grep "CPU=" "$TEMP_TIME_FILE" | sed 's/CPU=//')
        #MEM=$(grep "MEM=" "$TEMP_TIME_FILE" | sed 's/MEM=//')
        
        # Append the results (comma-separated) to the results file.
        echo "$ROBOTS,$rep,$ROW" >> "$RESULTS_FILE"
        rm -f "$TEMP_TIME_FILE"
    done
done

echo "All experiments complete. Results saved to $RESULTS_FILE."

