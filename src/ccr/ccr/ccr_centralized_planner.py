import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
from driving_swarm_messages.msg import BeliefState as BeliefStateMsg

class CentralizedPlanner(DrivingSwarmNode):

    def __init__(self):
        super().__init__('centralized_planner')
        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        self.robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value

        # Subscriber and Publisher dictionaries for each robot
        self.subscribers = {}
        self.publishers = {}

        for robot in self.robot_names:
            self.subscribers[robot] = {
                'state': self.create_subscription(Int32, f'/{robot}/nav/current_node', self.create_state_callback(robot), 10),
                'goal': self.create_subscription(Int32, f'/{robot}/nav/goal_node', self.create_goal_callback(robot), 10)
            }
            self.publishers[robot] = self.create_publisher(Int32MultiArray, f'/{robot}/nav/plan', 10)

	# initialize dictionary for state and goal information
        self.central_plan = {}
        for robot in self.robot_names:
            self.central_plan[robot_name]['state'] = -1
            self.central_plan[robot_name]['goal'] = -1

        # Set up timer to distribute plans periodically
        self.timer = self.create_timer(1.0, self.distribute_plans)

    def create_state_callback(self, robot_name):
        def callback(msg):
            # Handle state update from robot
            self.get_logger().info(f'Received state {msg.data} from {robot_name}')
            # Example of updating internal plan state
            self.central_plan[robot_name]['state'] = msg.data
        return callback

    def create_goal_callback(self, robot_name):
        def callback(msg):
            # Handle goal update from robot
            self.get_logger().info(f'Received goal {msg.data} from {robot_name}')
            # Example of updating internal plan goal
            self.central_plan[robot_name]['goal'] = msg.data
        return callback

    def distribute_plans(self):
        # Example logic to generate and distribute plans to all robots
        
        # check if all robots have a state and goal
        for robot in self.robot_names :
            if (central_plan[robot]['state']== -1 or central_plan[robot]['goal']== -1)
                return
        
        generate_plan_for_robots()
        
        for robot in self.robot_names:
            plan = central_plan[robot]['plan']
            msg = Int32MultiArray()
            msg.data = plan
            self.publishers[robot].publish(msg)
            self.get_logger().info(f'Sent plan {plan} to {robot}')

    def generate_plan_for_robots(self):
        # TODO : calculate plan and add them to the central_plan dictionary
           

def main(args=None):
    rclpy.init(args=args)
    node = CentralizedPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

