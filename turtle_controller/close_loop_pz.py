import rclpy, time, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import tf_transformations
import sys

class ClosedLoopControllerNode(Node):
    def __init__(self):
        super().__init__('closed_loop_controller')
        self.get_logger().info("Closed LoopNode Initialized...")
        self.create_timer(0.1, self.go_to_point)

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 1)

        self.create_subscription(Odometry, "/odom", self.pose_odom_callback, 1)


        self.sub = self.create_subscription(Pose, "/next_point", self.callback_points, 10)
        self.pub_arrived = self.create_publisher(Bool, "/arrived", 1)

        self.x = None
        self.y = None
        self.q = None

        self.x_d = 0.0
        self.y_d = 0.0

        self.arrived = Bool()
        self.action_finished = False

    def pose_odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Convert quaternion to Euler angles

        q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf_transformations.euler_from_quaternion(q)
        self.q = euler[2]


        
    def callback_points(self, msg):
        self.x_d = msg.x
        self.y_d = msg.y


    def go_to_point(self):
       Kv, Kw = 0.5, 2.0
       if self.x is not None:
           msg = Twist()

           #Diference in position
           Dx = self.x_d - self.x
           Dy = self.y_d - self.y

           

           # Calculating the distance between the turtle and the desired point
           distance = math.sqrt(Dx**2 + Dy**2)

           if distance > 0.1:
                # Calculating the angle to the desired point
                angle = math.atan2(Dy, Dx) - self.q

                # Normalize the angle to be between -pi and pi
                angle = math.atan2(math.sin(angle), math.cos(angle))

                #saturation of the linear and angular velocity 

        

                msg.linear.x = Kv if distance > 0.1 else 0.0
                msg.angular.z = Kw * angle

                #saturation of the linear and angular velocity
                if msg.linear.x > 0.2:
                    msg.linear.x = 0.2
                if msg.angular.z > 0.3:
                    msg.angular.z = 0.3
                if msg.angular.z < -0.3:
                    msg.angular.z = -0.3
                if msg.linear.x < -0.2:
                    msg.linear.x = -0.2

                self.arrived.data = False
                self.pub_arrived.publish(self.arrived)

            

           if distance < 0.1:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.action_finished = True
                self.arrived.data = True
                self.pub_arrived.publish(self.arrived)

           self.pub.publish(msg) 
           self.pub_cmd.publish(msg)   
  

    def pure_pursuit(self):
        L = 0.8
        if self.x is not None:
            msg = Twist()

            #Diference in position
            Dx = self.x_d - self.x
            Dy = self.y_d - self.y           
            # Calculating the distance between the turtle and the desired point
            distance = math.sqrt(Dx**2 + Dy**2)
            if distance > 0.1:
                cq = math.cos(self.q)
                sq = math.sin(self.q)

                msg.linear.x = Dx*cq + Dy*sq
                msg.angular.z = (1/L)*(Dy*cq - Dx*sq)
                self.arrived.data = False
                self.pub_arrived.publish(self.arrived)

            else:

                self.arrived.data = True
                self.pub_arrived.publish(self.arrived)

            self.pub.publish(msg)
            self.pub_cmd.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    sys.exit()

if __name__ == '__main__':
    main()