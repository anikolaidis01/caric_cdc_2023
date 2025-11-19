#!/bin/bash

# --- Configuration ---
iterations=10
scenario="mbs"
mission_duration=50  # seconds (must match your launch file)
total_uavs=4  # Total number of UAVs to record topics for
enable_video_recording=true  # Set to true/false to enable/disable video recording
log_base_dir=/media/dronesteam/"New Volume"/EUSOME_simulation_rosbagfiles

# --- Video Configuration (only used if enable_video_recording is true) ---
video_output_dir="$log_base_dir/videos"
video_duration=60  # seconds to record (adjust as needed)
video_resolution="1024x768" # Match your Xvfb resolution
video_framerate=25

# --- Lock file to prevent multiple instances ---
LOCK_FILE="/tmp/run_eusomeall.lock"
if [ -f "$LOCK_FILE" ]; then
    echo "Lock file $LOCK_FILE exists. Another instance might be running. Exiting."
    exit 1
fi
echo $$ > "$LOCK_FILE"

# --- Global Variables for Cleanup ---
MAIN_SCRIPT_PID=$$
LAUNCH_PID=""
ROSBAG_PID=""
VIDEO_PID="" # Added for video recording

# --- Function: Cleanup all processes and remove lock ---
cleanup_processes() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Received signal. Initiating cleanup (Script PID: $MAIN_SCRIPT_PID)..."
    
    # Kill the main ROS/Gazebo launch process (if running)
    if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Killing roslaunch process ($LAUNCH_PID)..."
        kill $LAUNCH_PID 2>/dev/null
        sleep 3
        kill -9 $LAUNCH_PID 2>/dev/null
    fi

    # Kill the rosbag recording process (if running)
    if [ -n "$ROSBAG_PID" ] && kill -0 "$ROSBAG_PID" 2>/dev/null; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Killing rosbag process ($ROSBAG_PID)..."
        kill $ROSBAG_PID 2>/dev/null
        sleep 3
        kill -9 $ROSBAG_PID 2>/dev/null
    fi

    # Kill the video recording process (if running AND recording is enabled)
    if [ "$enable_video_recording" = true ] && [ -n "$VIDEO_PID" ] && kill -0 "$VIDEO_PID" 2>/dev/null; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Killing video process ($VIDEO_PID)..."
        # Send SIGINT to ffmpeg for graceful shutdown
        kill -INT $VIDEO_PID 2>/dev/null
        sleep 5 # Wait for ffmpeg to finalize
        # Force kill if it doesn't stop
        if kill -0 "$VIDEO_PID" 2>/dev/null; then
            kill -9 $VIDEO_PID 2>/dev/null
        fi
        wait $VIDEO_PID 2>/dev/null
    elif [ "$enable_video_recording" = false ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Video recording is disabled. No video process to kill. (Script PID: $MAIN_SCRIPT_PID)"
    fi

    # Wait for the specific launch, rosbag, and video processes to finish
    wait $LAUNCH_PID 2>/dev/null
    wait $ROSBAG_PID 2>/dev/null
    if [ "$enable_video_recording" = true ] && [ -n "$VIDEO_PID" ]; then
        wait $VIDEO_PID 2>/dev/null
    fi

    # Kill any remaining related processes found by name
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Killing any remaining related processes..."
    pkill -f "roslaunch.*run_solution.launch" 2>/dev/null
    pkill -f "rosbag record" 2>/dev/null
    if [ "$enable_video_recording" = true ]; then
        pkill -f "ffmpeg.*x11grab" 2>/dev/null # Kill any stray ffmpeg x11grab processes only if recording was enabled
    fi
    pkill -f "gzserver" 2>/dev/null
    pkill -f "gzclient" 2>/dev/null
    pkill -f "rosmaster" 2>/dev/null
    pkill -f "roscore" 2>/dev/null
    pkill -f "explorer.py" 2>/dev/null
    pkill -f "explorer_path.py" 2>/dev/null
    pkill -f "gcs.py" 2>/dev/null
    pkill -f "photographer.py" 2>/dev/null
    pkill -f "keep_score.py" 2>/dev/null
    
    # *** NEW: Kill any rviz processes ***
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Killing any remaining rviz processes..."
    pkill -f "rviz\|RViz" 2>/dev/null

    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Killing any remaining octomap_server processes... (Script PID: $MAIN_SCRIPT_PID)"
    pkill -f "octomap_server" 2>/dev/null

    # kill unicon process
    pkill -f "unicon" 2>/dev/null
    # pkill -f "caric_mission" 2>/dev/null
    # Wait a bit more for processes to die gracefully if possible

    sleep 5

    # Final force kill attempt if anything persists
    pkill -9 -f "roslaunch.*run_solution.launch" 2>/dev/null
    pkill -9 -f "rosbag record" 2>/dev/null
    if [ "$enable_video_recording" = true ]; then
        pkill -9 -f "ffmpeg.*x11grab" 2>/dev/null # Force kill only if recording was enabled
    fi
    pkill -9 -f "gzserver\|gzclient\|rosmaster\|roscore" 2>/dev/null
    pkill -9 -f "explorer.py\|explorer_path.py\|gcs.py" 2>/dev/null

    # *** NEW: Force kill any remaining rviz processes ***
    pkill -9 -f "rviz\|RViz" 2>/dev/null
    # *** NEW: Force kill octomap_server if it persists ***
    pkill -9 -f "octomap_server" 2>/dev/null
    pkill -9 -f "unicon" 2>/dev/null
    
    # Ensure Gazebo processes are *really* dead before next iteration
    # This is crucial to prevent "model already exists" errors
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Ensuring Gazebo server/client are dead..."
    sleep 3 # Brief pause
    pkill -9 -f "gzserver\|gzclient" 2>/dev/null # Force kill again
    sleep 3 # Brief pause

    # Kill any remaining nodes (this might fail if roscore is dead, which is okay)
    # Use a timeout for rosnode kill in case roscore is dead but node list hangs
    timeout 5s rosnode kill -a 2>/dev/null || echo "[$(date '+%Y-%m-%d %H:%M:%S')] rosnode kill -a timed out or failed, likely no ROS master."

    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Cleanup completed."
}

# --- Function: Handle script exit (including errors) ---
cleanup_on_exit() {
    local exit_code=$?
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Script (PID: $MAIN_SCRIPT_PID) exiting with code $exit_code."
    cleanup_processes
    # Remove the lock file
    rm -f "$LOCK_FILE"
    exit $exit_code
}

# --- Set traps for signals and exit ---
# SIGINT (Ctrl+C), SIGTERM (kill command), SIGHUP (terminal closed)
trap 'cleanup_processes; exit 130' SIGINT SIGTERM SIGHUP
# EXIT: Runs on any exit (success, error, signal)
trap cleanup_on_exit EXIT

# --- Ensure log base dir and video output dir exist ---
mkdir -p "$log_base_dir"
if [ "$enable_video_recording" = true ]; then
    mkdir -p "$video_output_dir"
fi

# --- Source ROS once at the top (more efficient) ---
source ~/eusome/devel/setup.bash

# --- Function to generate UAV name ---
get_uav_name() {
    local index=$1
    if [ $index -eq 1 ]; then
        echo "jurong"
    elif [ $index -eq 2 ]; then
        echo "raffles"
    elif [ $index -ge 3 ]; then
        printf "expl_uav_%02d" $((index))
    else
        echo "unknown"
    fi
}

# --- Function to generate all topic arrays ---
generate_topics() {
    local num_uavs=$1
    local odom_topics=()
    local travel_metric_topics=()
    local total_metric_topics=()
    
    for ((i=1; i<=num_uavs; i++)); do # This 'i' is local to the function
        uav_name=$(get_uav_name $i)
        odom_topics+=("/$uav_name/ground_truth/odometry")
        travel_metric_topics+=("/$uav_name/area_travel_metrics")
        total_metric_topics+=("/$uav_name/total_travel_metrics")
    done
    
    export ODOM_TOPICS="${odom_topics[*]}"
    export TRAVEL_METRIC_TOPICS="${travel_metric_topics[*]}"
    export TOTAL_METRIC_TOPICS="${total_metric_topics[*]}"
    
    local all_topics=()
    all_topics+=("${odom_topics[@]}")
    all_topics+=("${travel_metric_topics[@]}")
    all_topics+=("${total_metric_topics[@]}")
    
    export ALL_TOPICS_FOR_ROSBAG="${all_topics[*]}"
}

# --- Main Loop ---
# Use a different, more unique variable name for the main loop to avoid potential conflicts
for sim_iter in $(seq 1 $iterations); do
    echo "=== Starting simulation iteration $sim_iter / $iterations (Script PID: $MAIN_SCRIPT_PID) ==="
    iter_log_dir="$log_base_dir/run_$sim_iter"
    mkdir -p "$iter_log_dir"

    # --- PRE-LOOP: Reset PIDs ---
    LAUNCH_PID=""
    ROSBAG_PID=""
    VIDEO_PID="" # Reset video PID

    # --- CLEANUP FROM PREVIOUS RUN (Enhanced Aggressive) ---
    # Only run aggressive cleanup for iterations *after* the first one.
    # The first run should ideally start with a clean environment.
    # If the script was interrupted before, this helps, but avoid killing itself on first start.
    if [ $sim_iter -gt 1 ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] AGGRESSIVE CLEANUP BEFORE ITERATION $sim_iter (Script PID: $MAIN_SCRIPT_PID)..."
        
        # Kill any potential leftover roslaunch processes from *any* solution launch
        pkill -f "roslaunch.*run_solution.launch" 2>/dev/null
        # Kill rosbag recorders
        pkill -f "rosbag record" 2>/dev/null
        # Kill video recorders (only if recording was enabled)
        if [ "$enable_video_recording" = true ]; then
            pkill -f "ffmpeg.*x11grab" 2>/dev/null
        fi
        # Kill Gazebo server and client
        pkill -f "gzserver" 2>/dev/null
        pkill -f "gzclient" 2>/dev/null
        # Kill ROS master/core
        pkill -f "rosmaster" 2>/dev/null
        pkill -f "roscore" 2>/dev/null
        # Kill script-launched ROS nodes
        pkill -f "explorer.py\|explorer_path.py\|gcs.py\|photographer.py\|keep_score.py" 2>/dev/null

        # Wait briefly for graceful shutdown attempts
        sleep 5

        # Force kill anything that survived the first round
        pkill -9 -f "gzserver\|gzclient\|rosmaster\|roscore" 2>/dev/null
        if [ "$enable_video_recording" = true ]; then
            pkill -9 -f "ffmpeg.*x11grab" 2>/dev/null
        fi
        pkill -9 -f "explorer.py\|explorer_path.py\|gcs.py" 2>/dev/null
        # Also force kill any remaining roslaunch/record processes
        pkill -9 -f "roslaunch.*run_solution.launch" 2>/dev/null
        pkill -9 -f "rosbag record" 2>/dev/null

        # Wait longer for force kills to take effect
        sleep 8

        # *** NEW: Ensure Gazebo is *completely* dead ***
        # Check for any remaining gzserver/gzclient processes
        if pgrep -f "gzserver\|gzclient" > /dev/null; then
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: Gazebo processes still found after force kill. PID list:"
            pgrep -f "gzserver\|gzclient"
            # Try one more time with killall if available
            killall -9 gzserver gzclient 2>/dev/null
            sleep 3
        fi

        # *** NEW: Force stop any existing ROS master/core ***
        # Use a dummy launch to kill any existing rosmaster/roscore
        # This command might fail if no master was running, which is fine.
        timeout 10s roslaunch --pid 2>/dev/null | xargs kill -9 2>/dev/null || true
        # Alternatively, just kill any remaining rosmaster/roscore PIDs directly again
        pkill -9 -f "rosmaster\|roscore" 2>/dev/null
        sleep 3 # Give it time to die

        # *** NEW: Brief pause after cleanup to let system settle ***
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Brief pause after cleanup for iteration $sim_iter (Script PID: $MAIN_SCRIPT_PID)..."
        sleep 5

        # Verify no critical processes remain (optional, for debugging)
        # ps aux | grep -E "(gzserver|gzclient|rosmaster|roscore)" | grep -v grep
    else
        # For the very first iteration, DO NOT perform any pkill-based cleanup.
        # Assume the user has started with a clean environment or will handle initial state manually.
        # The trap-based cleanup at the end of the script (or on error) should handle the first run's processes.
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] First iteration ($sim_iter). Assuming clean environment, skipping pre-launch pkill. (Script PID: $MAIN_SCRIPT_PID)"
        # Optional: Add a very basic check if needed, but avoid pkill.
        # if pgrep -f "gzserver" > /dev/null; then
        #    echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: gzserver found before first launch. This might cause issues."
        #    # Decide whether to exit or proceed based on tolerance for risk.
        #    # exit 1 # Uncomment to exit if a Gazebo server is found.
        # fi
    fi


    # --- GENERATE TOPICS FOR ALL UAVS ---
    generate_topics $total_uavs
    read -ra TOPICS_ARRAY <<< "$ALL_TOPICS_FOR_ROSBAG"

    # --- LAUNCH SIMULATION ---
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Launching simulation for iteration $sim_iter (Script PID: $MAIN_SCRIPT_PID)..."
    roslaunch kios_solution run_solution.launch scenario:="$scenario" num_explorers:="$total_uavs" &
    LAUNCH_PID=$!

    # Wait a bit for ROS nodes to start appearing, check if launch failed quickly
    sleep 10
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: roslaunch process ($LAUNCH_PID) seems to have died quickly for iteration $sim_iter. Launch failed."
        # Perform cleanup for this failed launch attempt before continuing
        cleanup_processes
        LAUNCH_PID="" # Reset PID as it's dead
        # Continue to next iteration, hoping state was reset
        continue
    fi

    # --- START ROSBAG RECORDING (DYNAMIC) ---
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Starting rosbag recording for run $sim_iter (Script PID: $MAIN_SCRIPT_PID)..."
    rosbag record -o "$iter_log_dir/mission_data" "${TOPICS_ARRAY[@]}" &
    ROSBAG_PID=$!

    # --- WAIT FOR MISSION MANAGER TO START ---
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Waiting for /mission_manager node to appear for iteration $sim_iter (Script PID: $MAIN_SCRIPT_PID)..."
    if ! timeout 60s bash -c 'while ! rosnode list 2>/dev/null | grep -q "/mission_manager"; do sleep 2; done'; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] ERROR: /mission_manager did not start within 60 seconds for iteration $sim_iter."
        # Trigger cleanup manually if wait fails, then continue to next iteration
        cleanup_processes
        continue # Skip the rest of this loop iteration
    fi

    echo "[$(date '+%Y-%m-%d %H:%M:%S')] /mission_manager is running for iteration $sim_iter. Mission duration: ${mission_duration}s (Script PID: $MAIN_SCRIPT_PID)"

    # --- VIDEO RECORDING SECTION ---
        # --- START ROSBAG RECORDING (DYNAMIC) ---
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Starting rosbag recording for run $sim_iter (Script PID: $MAIN_SCRIPT_PID)..."
    rosbag record -o "$iter_log_dir/mission_data" "${TOPICS_ARRAY[@]}" &
    ROSBAG_PID=$!

    # --- WAIT FOR MISSION MANAGER TO START ---
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Waiting for /mission_manager node to appear for iteration $sim_iter (Script PID: $MAIN_SCRIPT_PID)..."
    if ! timeout 60s bash -c 'while ! rosnode list 2>/dev/null | grep -q "/mission_manager"; do sleep 2; done'; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] ERROR: /mission_manager did not start within 60 seconds for iteration $sim_iter."
        # Trigger cleanup manually if wait fails, then continue to next iteration
        cleanup_processes
        continue # Skip the rest of this loop iteration
    fi

    echo "[$(date '+%Y-%m-%d %H:%M:%S')] /mission_manager is running for iteration $sim_iter. Mission duration: ${mission_duration}s (Script PID: $MAIN_SCRIPT_PID)"

    # --- VIDEO RECORDING SECTION (NEW) ---
    if [ "$enable_video_recording" = true ]; then
        # Ensure the video output directory exists
        mkdir -p "$video_output_dir"

        # Construct the video filename with the correct path and a unique name per iteration
        video_filename="$video_output_dir/video_run_${sim_iter}_$(date +%Y%m%d_%H%M%S).mp4"
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Starting video recording for iteration $sim_iter: $video_filename (Script PID: $MAIN_SCRIPT_PID)"

        # Check if the video output directory exists
        if [ ! -d "$video_output_dir" ]; then
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] ERROR: Video output directory does not exist: $video_output_dir (Script PID: $MAIN_SCRIPT_PID)"
            # Create it (recommended)
            mkdir -p "$video_output_dir"
            if [ $? -ne 0 ]; then
                echo "[$(date '+%Y-%m-%d %H:%M:%S')] ERROR: Failed to create video output directory: $video_output_dir (Script PID: $MAIN_SCRIPT_PID)"
                VIDEO_PID="" # Indicate failure
            else
                echo "[$(date '+%Y-%m-%d %H:%M:%S')] Created video output directory: $video_output_dir (Script PID: $MAIN_SCRIPT_PID)"
                # Launch the video recorder with the specified filename
                # Use 'env' to ensure DISPLAY is passed if needed (important if using Xvfb)
                env DISPLAY="$DISPLAY" nice -n 10 ionice -c 3 ffmpeg -y -f x11grab -video_size $video_resolution -framerate $video_framerate -i $DISPLAY -c:v libx264 -preset ultrafast -crf 28 -pix_fmt yuv420p -t $video_duration -r $video_framerate "$video_filename" > "$video_filename.log" 2>&1 &
                VIDEO_PID=$!
                echo "[$(date '+%Y-%m-%d %H:%M:%S')] Video recording command issued with PID $VIDEO_PID for iteration $sim_iter. Log: $video_filename.log (Script PID: $MAIN_SCRIPT_PID)"

                # Optional: Brief check if the process actually started
                sleep 2
                if ! kill -0 $VIDEO_PID 2>/dev/null; then
                    echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: Video recording process ($VIDEO_PID) seems to have exited quickly or failed to start for iteration $sim_iter. Check log: $video_filename.log (Script PID: $MAIN_SCRIPT_PID)"
                    # Capture exit code if possible
                    wait $VIDEO_PID 2>/dev/null
                    FFMPEG_EXIT_CODE=$?
                    echo "[$(date '+%Y-%m-%d %H:%M:%S')] ffmpeg exit code was: $FFMPEG_EXIT_CODE for iteration $sim_iter. (Script PID: $MAIN_SCRIPT_PID)"
                    VIDEO_PID=""
                else
                    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Video recording process ($VIDEO_PID) seems to be running for iteration $sim_iter. (Script PID: $MAIN_SCRIPT_PID)"
                fi
            fi
        else
            # Directory exists, proceed with launch
            # Use 'env' to ensure DISPLAY is passed if needed (important if using Xvfb)
            env DISPLAY="$DISPLAY" nice -n 10 ionice -c 3 ffmpeg -y -f x11grab -video_size $video_resolution -framerate $video_framerate -i $DISPLAY -c:v libx264 -preset ultrafast -crf 28 -pix_fmt yuv420p -t $video_duration -r $video_framerate "$video_filename" > "$video_filename.log" 2>&1 &
            VIDEO_PID=$!
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] Video recording command issued with PID $VIDEO_PID for iteration $sim_iter. Log: $video_filename.log (Script PID: $MAIN_SCRIPT_PID)"

            # Optional: Brief check if the process actually started
            sleep 2
            if ! kill -0 $VIDEO_PID 2>/dev/null; then
                echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: Video recording process ($VIDEO_PID) seems to have exited quickly or failed to start for iteration $sim_iter. Check log: $video_filename.log (Script PID: $MAIN_SCRIPT_PID)"
                # Capture exit code if possible
                wait $VIDEO_PID 2>/dev/null
                FFMPEG_EXIT_CODE=$?
                echo "[$(date '+%Y-%m-%d %H:%M:%S')] ffmpeg exit code was: $FFMPEG_EXIT_CODE for iteration $sim_iter. (Script PID: $MAIN_SCRIPT_PID)"
                VIDEO_PID=""
            else
                echo "[$(date '+%Y-%m-%d %H:%M:%S')] Video recording process ($VIDEO_PID) seems to be running for iteration $sim_iter. (Script PID: $MAIN_SCRIPT_PID)"
            fi
        fi
    else
        # Video recording is disabled
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Video recording is disabled for iteration $sim_iter. (Script PID: $MAIN_SCRIPT_PID)"
        VIDEO_PID="" # Explicitly set to empty if disabled
    fi # <--- THIS 'fi' WAS MISSING! It closes the 'if [ "$enable_video_recording" = true ];' block.

    # Optional: Add a brief delay before starting the main wait loop
    # This allows video recording (if enabled) to stabilize if needed
    sleep 5

    # --- WAIT FOR MISSION TO FINISH OR TIME OUT ---
    # This line and everything below it should be OUTSIDE the video recording 'if' block
    start_time=$(date +%s)
    max_wait=$((mission_duration + 50)) # Add buffer
    count=0
    while rosnode list 2>/dev/null | grep -q "/mission_manager" && [ $count -lt $max_wait ]; do
        sleep 1
        ((count++))
        # Optional: Log progress every 60 seconds
        if [ $((count % 60)) -eq 0 ]; then
            elapsed=$(( $(date +%s) - start_time ))
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] Mission for iteration $sim_iter still running... wait count: ${count}/${max_wait}, elapsed: ${elapsed}s (Script PID: $MAIN_SCRIPT_PID)"
        fi
    done

    elapsed_time=$(( $(date +%s) - start_time ))
    if [ $count -ge $max_wait ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: Mission for iteration $sim_iter did not finish in time (waited $count s, elapsed $elapsed_time s)."
    else
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Mission for iteration $sim_iter completed successfully after ${elapsed_time}s."
    fi

    # --- SHUTDOWN LAUNCH, ROSBAG, AND VIDEO (Enhanced) ---
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Shutting down simulation for iteration $sim_iter (Script PID: $MAIN_SCRIPT_PID)..."
    # Kill the main launch process
    if [ -n "$LAUNCH_PID" ]; then
        kill $LAUNCH_PID 2>/dev/null
        sleep 5 # Give it time to shut down gracefully
        kill -9 $LAUNCH_PID 2>/dev/null # Force if necessary
        wait $LAUNCH_PID 2>/dev/null # Wait for it to finish
    fi
    # Kill the rosbag process
    if [ -n "$ROSBAG_PID" ]; then
        kill $ROSBAG_PID 2>/dev/null
        sleep 3
        kill -9 $ROSBAG_PID 2>/dev/null
        wait $ROSBAG_PID 2>/dev/null
    fi
    # Kill the VIDEO recording process (NEW)
    if [ -n "$VIDEO_PID" ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Killing video recording process ($VIDEO_PID) for iteration $sim_iter... (Script PID: $MAIN_SCRIPT_PID)"
        # ffmpeg is not a ROS node, use SIGINT for graceful shutdown
        kill -INT $VIDEO_PID 2>/dev/null
        sleep 5 # Give ffmpeg time to finalize the video file
        # Force kill if it doesn't stop gracefully
        if kill -0 "$VIDEO_PID" 2>/dev/null; then
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] Video recording process ($VIDEO_PID) did not stop gracefully, force killing for iteration $sim_iter. (Script PID: $MAIN_SCRIPT_PID)"
            kill -9 $VIDEO_PID 2>/dev/null
        fi
        wait $VIDEO_PID 2>/dev/null # Wait for it to finish dying
    else
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] No video recording process to kill for iteration $sim_iter. (Script PID: $MAIN_SCRIPT_PID)"
    fi

    # Perform final cleanup for this iteration
    cleanup_processes
    # Reset PIDs after cleanup within the loop
    LAUNCH_PID=""
    ROSBAG_PID=""
    VIDEO_PID="" # Reset video PID

    # Brief pause before next iteration to ensure cleanup is complete
    sleep 5

    # Construct the video filename for the log message, considering the flag
    if [ "$enable_video_recording" = true ] && [ -n "$video_filename" ]; then
        video_log_msg="Video saved to: $video_filename"
    elif [ "$enable_video_recording" = false ]; then
        video_log_msg="Video recording was disabled."
    else
        # This case handles if video recording was enabled but the filename wasn't set due to an error
        video_log_msg="Video recording failed or filename not set."
    fi

    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Iteration $sim_iter finished (Script PID: $MAIN_SCRIPT_PID). Logs saved to: $iter_log_dir/, $video_log_msg"
    echo ""
done

# Final message depends on whether video recording was enabled
if [ "$enable_video_recording" = true ]; then
    echo "All $iterations simulations completed (Script PID: $MAIN_SCRIPT_PID). Videos saved to: $video_output_dir"
else
    echo "All $iterations simulations completed (Script PID: $MAIN_SCRIPT_PID). Video recording was disabled."
fi

# The 'trap cleanup_on_exit EXIT' will handle final cleanup when the script ends normally.
# Remove the lock file at the very end of successful execution
rm -f "$LOCK_FILE"