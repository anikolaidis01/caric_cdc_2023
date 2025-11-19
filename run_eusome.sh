!#/bin/bash/

killall -9 gzclient
killall -9 gzserver
killall -9 explorer.py
killall -9 explorer_path.py
killall -9 photographer.py
killall -9 photographer_path.py
killall -9 gcs.py
killall -9 keep_score.py


source ~/eusome/devel/setup.bash
rosnode kill -a

# python /home/dronesteam/ws_caric/src/kios_solution/scripts/keep_score.py &

roslaunch kios_solution run_solution.launch scenario:=$1
#roslaunch kios_solution test_hangar.launch
