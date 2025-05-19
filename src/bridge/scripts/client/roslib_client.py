#!/usr/bin/env python3

import roslibpy
import roslibpy.actionlib

# Host IP changes, the port is always 3201
client = roslibpy.Ros(host="localhost", port=3201)
client.run()

# Define the action client
action = roslibpy.actionlib.ActionClient(
    client, "/test_action", "bridge_msgs/action/BridgeAction"
)

# Define the goal message with correct parameter
goal = roslibpy.actionlib.Goal(action, roslibpy.Message({"number_of_seconds": 10}))


# Optional: Set up feedback callback
def feedback_callback(feedback):
    seconds_elapsed = feedback.get("seconds_elapsed", 0)
    print(f"Received feedback: {seconds_elapsed} seconds elapsed")


goal.on("feedback", feedback_callback)

# Send the goal
goal.send()

# Wait for result (no timeout)
result = goal.wait()

# Clean up
action.dispose()

# Print results
print("Result message: {}".format(result.get("message", "No message")))
