import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
# from mavros_msgs.srv import CommandBool, SetMode, CommandTOL # MAVROS removed
# from mavros_msgs.msg import State # MAVROS removed
import time
import json
import os
from datetime import datetime
import threading
import base64

class MissionControl(Node):
    def __init__(self):
        super().__init__("mission_control")
        self.get_logger().info("Mission Control Node started")
        self.get_logger().info("Initializing MissionControlNode.")

        self.get_logger().info("Creating command_publisher.")
        self.command_publisher = self.create_publisher(String, "/drone_inspetor/mission_command", 10)
        
        self.get_logger().info("Creating detection_subscriber.")
        self.detection_subscriber = self.create_subscription(
            String, # Assuming a String message for simplicity, could be custom message
            "/drone_inspetor/object_detection",
            self.object_detection_callback,
            10
        )
        self.get_logger().info("Creating fsm_state_publisher.")
        self.fsm_state_publisher = self.create_publisher(String, "/drone_inspetor/fsm_state", 10)

        self.log_dir = ""
        self.detected_objects = []
        self.mission_active = False
        self.current_pose = PoseStamped()

        # Subscribe to local position for drone's current pose
        self.get_logger().info("Creating local_pose_sub subscriber.")
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose", # Assuming MAVROS is used for pose, but this topic needs to be bridged from Gazebo
            self.local_pose_cb,
            10
        )
        self.get_logger().info("MissionControlNode initialization complete.")

    def local_pose_cb(self, msg):
        self.get_logger().info("Received local pose message in local_pose_cb.")
        self.current_pose = msg
        self.get_logger().info(f"Current pose updated: x={self.current_pose.pose.position.x}, y={self.current_pose.pose.position.y}, z={self.current_pose.pose.position.z}")

    def send_command(self, command):
        self.get_logger().info(f"Attempting to send command: {command}")
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Sent command: {command}")
        self.get_logger().info(f"Command '{command}' published.")

    def publish_fsm_state(self, state):
        self.get_logger().info(f"Attempting to publish FSM state: {state}")
        msg = String()
        msg.data = state
        self.fsm_state_publisher.publish(msg)
        self.get_logger().info(f"FSM state '{state}' published.")

    def start_inspection_mission(self):
        self.get_logger().info("start_inspection_mission called.")
        if self.mission_active:
            self.get_logger().warn("Mission already active. Ignoring start command.")
            return

        self.mission_active = True
        self.publish_fsm_state("Executando Inspeção")
        self.get_logger().info("Starting inspection mission...")
        self.get_logger().info("Creating log directory.")
        self.log_dir = os.path.join("logs", f"missao_{datetime.now().strftime("%Y%m%d_%H%M%S")}")
        os.makedirs(self.log_dir, exist_ok=True)
        self.detected_objects = []

        # Arm and Takeoff (commands sent to drone_node)
        self.get_logger().info("Sending 'start_inspection' command to drone_node.")
        self.send_command("start_inspection")

        # Simulate automated sweep (zig-zag or circular)
        self.get_logger().info("Automated sweep placeholder: Drone is flying in a 1000m² area...")
        self.get_logger().info("Simulating flight time (20 seconds delay).")
        time.sleep(20) # Simulate flight time for inspection
        self.get_logger().info("Simulated flight time complete.")

        self.get_logger().info("Automated sweep placeholder: Mission complete. Returning to base.")
        self.get_logger().info("Sending 'return_to_base' command to drone_node.")
        self.send_command("return_to_base")
        self.publish_fsm_state("Retornando à base")

        self.mission_active = False
        self.publish_fsm_state("Pronto")
        self.get_logger().info("Inspection mission finished.")
        self.get_logger().info("start_inspection_mission finished.")

    def stop_inspection_mission(self):
        self.get_logger().info("stop_inspection_mission called.")
        if not self.mission_active:
            self.get_logger().warn("No active mission to stop. Ignoring stop command.")
            return
        self.get_logger().info("Stopping inspection mission.")
        self.get_logger().info("Sending 'stop_inspection' command to drone_node.")
        self.send_command("stop_inspection")
        self.publish_fsm_state("Pausando Inspeção")
        self.mission_active = False
        self.get_logger().info("stop_inspection_mission finished.")

    def continue_inspection_mission(self):
        self.get_logger().info("continue_inspection_mission called.")
        if self.mission_active:
            self.get_logger().warn("Mission already active. Ignoring continue command.")
            return
        self.get_logger().info("Continuing inspection mission.")
        self.get_logger().info("Sending 'continue_inspection' command to drone_node.")
        self.send_command("continue_inspection")
        self.publish_fsm_state("Executando Inspeção")
        self.mission_active = True
        self.get_logger().info("continue_inspection_mission finished.")

    def return_to_base_mission(self):
        self.get_logger().info("return_to_base_mission called.")
        if not self.mission_active:
            self.get_logger().warn("No active mission to return from. Ignoring return command.")
            return
        self.get_logger().info("Returning to base.")
        self.get_logger().info("Sending 'return_to_base' command to drone_node.")
        self.send_command("return_to_base")
        self.publish_fsm_state("Retornando à base")
        self.mission_active = False
        self.get_logger().info("return_to_base_mission finished.")

    def object_detection_callback(self, msg):
        self.get_logger().info("Received object detection message in object_detection_callback.")
        try:
            detections = json.loads(msg.data)
            self.get_logger().info(f"Decoded {len(detections)} detections.")
            for detection_data in detections:
                detection_data["timestamp"] = str(datetime.now())
                detection_data["gps_location"] = {
                    "latitude": self.current_pose.pose.position.x, # Placeholder for actual GPS conversion
                    "longitude": self.current_pose.pose.position.y,
                    "altitude": self.current_pose.pose.position.z
                }
                self.detected_objects.append(detection_data)
                self.get_logger().info(f"Detected object: {detection_data.get("label")} at {detection_data.get("gps_location")}")
                self.get_logger().info(f"Detection added to list: {detection_data.get("label")}")

                # Save image if available in detection_data
                image_base64 = detection_data.get("image_base64", None)
                if image_base64:
                    self.get_logger().info("Saving detection image.")
                    image_filename = os.path.join(self.log_dir, f"detection_{len(self.detected_objects)}_{detection_data.get("label")}.png")
                    with open(image_filename, "wb") as f:
                        f.write(base64.b64decode(image_base64))
                    self.get_logger().info(f"Saved detection image: {image_filename}")
                    self.get_logger().info("Detection image saved.")

            # Save all detection data to a single JSON file for the mission
            self.get_logger().info("Saving all detection data to JSON file.")
            with open(os.path.join(self.log_dir, "detections.json"), "w") as f:
                json.dump(self.detected_objects, f, indent=4)
            self.get_logger().info("Detection data saved to JSON file.")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON from object detection message: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in object_detection_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()

    # Example of how to trigger mission commands from the dashboard
    # These would typically be called by the dashboard's button callbacks
    # For testing, you can uncomment and call them directly:
    # node.start_inspection_mission()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


