#!/usr/bin/env python
import rospy
import re
from visualization_msgs.msg import Marker
import os
import signal

def callback(msg):
    # The Marker message is expected to have a 'text' field containing our time info.
    text = msg.text  # typically, Marker.text contains the string message.
    # rospy.loginfo("Received marker text: %s", text)
    
    # Regex pattern to extract the first number after "Time:" followed by optional spaces and then "s"
    pattern = r"Time:\s*([\d\.]+)\s*s"
    match = re.search(pattern, text)
    
    if match:
        elapsed_time = float(match.group(1))
        # rospy.loginfo("Extracted elapsed time: %.3f s", elapsed_time)
        mission_duration = rospy.get_param('~mission_duration', 500.0)
        if elapsed_time >= mission_duration:
            os.kill(os.getppid(), signal.SIGINT)
            # rospy.loginfo("Mission duration reached (%.3f s). Initiating shutdown.", elapsed_time)
            # rospy.signal_shutdown("Mission duration reached")
            os.kill(os.getppid(), signal.SIGINT)
    else:
        rospy.logwarn("Could not extract time from marker text: %s", text)

def main():
    rospy.init_node('viz_time_monitor')
    rospy.loginfo("Starting viz_time_monitor node.")
    rospy.Subscriber("viz_score_totalled_time", Marker, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
