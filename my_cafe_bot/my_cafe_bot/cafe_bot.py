import rclpy
from rclpy.node import Node
import time
import threading
import sys
import select
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class CafeBot(Node):
    def __init__(self):
        super().__init__('cafe_bot')
        self.navigator = BasicNavigator()

        # Declare waypoints
        self.declare_parameter('home', [3.406, 0.736])
        self.declare_parameter('kitchen', [3.715, 2.450])
        self.declare_parameter('table1', [7.184, 2.739])
        self.declare_parameter('table2', [8.051, 0.427])
        self.declare_parameter('table3', [10.730, 1.626])

        # Retrieve waypoints
        self.waypoints = {
            'home': self.get_parameter('home').get_parameter_value().double_array_value,
            'kitchen': self.get_parameter('kitchen').get_parameter_value().double_array_value,
            'table1': self.get_parameter('table1').get_parameter_value().double_array_value,
            'table2': self.get_parameter('table2').get_parameter_value().double_array_value,
            'table3': self.get_parameter('table3').get_parameter_value().double_array_value,
        }

        self.orders = []  # List to store selected tables
        self.user_skip = set()  # Tracks tables where 0 was pressed
        self.current_task = None
        self.order_cancelled = False  # Flag to check if 'C' was pressed

    def take_order(self):
        """Takes table orders from the user and waits for '9' to start the robot."""
        print("\nCafeBot Order System:")
        print("Press 1 for Table 1")
        print("Press 2 for Table 2")
        print("Press 3 for Table 3")
        print("Press 9 to start the robot")
        print("Press C to cancel and return home")

        while True:
            user_input = input("Enter order (or 9 to start, C to cancel): ")
            if user_input == "9":
                if self.orders:
                    break
                else:
                    print("No tables selected. Please select at least one table before pressing 9.")
            elif user_input.lower() == "c":
                print("Order cancelled. Returning to home position.")
                self.order_cancelled = True
                self.process_cancellation()
                return
            elif user_input in ["1", "2", "3"]:
                table = f"table{user_input}"
                if table not in self.orders:
                    self.orders.append(table)
                    print(f"Order added for {table}.")
                else:
                    print(f"Table {user_input} is already selected.")
            else:
                print("Invalid input. Please enter 1, 2, 3, 9, or C.")

        print("\nStarting order processing...\n")
        self.process_orders()

    def process_orders(self):
        """Processes the selected orders one by one."""
        if not self.orders:
            print("No orders received.")
            return

        # Move to the kitchen first if no 0 has been pressed before
        if not self.user_skip:
            self.move_to_location("kitchen")
            time.sleep(2)  # Simulate food pickup

        # Deliver to tables
        for table in self.orders:
            if self.order_cancelled:
                return  # Stop processing if cancellation happened

            self.current_task = table
            self.move_to_location(table)
            self.wait_at_table(table)

        # Decide final return location
        if len(self.user_skip) == len(self.orders):
            self.move_to_location("home")
        else:
            self.move_to_location("kitchen")
            self.move_to_location("home")

        self.current_task = None  # Task completed
        print("\nAll orders delivered. CafeBot is ready for new orders!")

    def process_cancellation(self):
        """Handles the order cancellation process."""
        self.move_to_location("kitchen")
        self.move_to_location("home")
        self.order_cancelled = False  # Reset flag after returning home
        print("\nOrder cancelled. CafeBot is now at home and ready for new orders!")

    def wait_at_table(self, table):
        """Waits 10 seconds at a table unless '0' is pressed to move immediately."""
        print(f"\nArrived at {table}. Waiting for 10 seconds...")
        print("Press 0 to confirm food placement and move immediately.")

        start_time = time.time()
        while time.time() - start_time < 10:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == "0":
                    print(f"Food placed at {table}. Moving to the next location...")
                    self.user_skip.add(table)
                    return

        print(f"Wait time over. Moving to next location...")

    def move_to_location(self, location):
        """Moves robot to the given waypoint location asynchronously."""
        if location not in self.waypoints:
            print(f"Unknown location: {location}")
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.waypoints[location][0]
        goal_pose.pose.position.y = self.waypoints[location][1]
        goal_pose.pose.orientation.w = 1.0  # Assume no rotation needed

        print(f"\nMoving to {location} at {self.waypoints[location]}...")

        # Send navigation goal asynchronously
        self.navigator.goToPose(goal_pose)

        # Wait for the navigation task to complete
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)  # Process ROS 2 callbacks while waiting

        print(f"\nArrived at {location}!")
def main(args=None):
    rclpy.init(args=args)
    node = CafeBot()

    # Create an executor to manage ROS 2 callbacks properly
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # Start the order-taking logic in a separate thread
    order_thread = threading.Thread(target=node.take_order, daemon=True)
    order_thread.start()

    # Run the ROS 2 executor in the main thread
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
