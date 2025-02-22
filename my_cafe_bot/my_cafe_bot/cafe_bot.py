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
        while True:
            self.orders.clear()  # Reset previous orders after returning home
            print("\nCafeBot Order System:")
            print("Press 1 for Table 1")
            print("Press 2 for Table 2")
            print("Press 3 for Table 3")
            print("Press 7 to start moving to tables")
            print("Press 9 to start the robot and wait at the kitchen")
            print("Press C to cancel and return home")

            move_to_tables = False

            while True:
                user_input = input("Enter order (7 to move to tables, 9 to wait at kitchen, C to cancel): ")
                if user_input == "7":
                    if self.orders:
                        move_to_tables = True
                        break
                    else:
                        print("No tables selected. Please select at least one table before pressing 7.")
                elif user_input == "9":
                    print("Moving to the kitchen and waiting for further instructions.")
                    self.move_to_location("kitchen")
                    if not self.wait_for_user_input(10, "7"):
                        print("No input received. Returning to home.")
                        self.move_to_location("home")
                        break  # Ask for the next order after returning home
                    move_to_tables = True  # If 7 is pressed, continue to tables
                    break
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
                    print("Invalid input. Please enter 1, 2, 3, 7, 9, or C.")

            if move_to_tables:
                print("\nStarting order processing...")
                self.process_orders()

    def wait_for_user_input(self, timeout, expected_input):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).strip()
                if key == expected_input:
                    return True
        return False

    def process_orders(self):
        if not self.orders:
            print("No orders received.")
            return

        self.move_to_location("kitchen")
        time.sleep(2)

        for table in self.orders:
            if self.order_cancelled:
                return

            self.current_task = table
            self.move_to_location(table)
            self.wait_at_table(table)

        self.move_to_location("home")
        self.current_task = None
        print("\nAll orders delivered. CafeBot is ready for new orders!")
        self.take_order()

    def wait_at_table(self, table):
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
        if location not in self.waypoints:
            print(f"Unknown location: {location}")
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.waypoints[location][0]
        goal_pose.pose.position.y = self.waypoints[location][1]
        goal_pose.pose.orientation.w = 1.0

        print(f"\nMoving to {location} at {self.waypoints[location]}...")

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        print(f"\nArrived at {location}!")


def main(args=None):
    rclpy.init(args=args)
    node = CafeBot()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    order_thread = threading.Thread(target=node.take_order, daemon=True)
    order_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
