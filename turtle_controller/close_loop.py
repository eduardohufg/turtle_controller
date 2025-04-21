import rclpy, time, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool
from std_msgs.msg import String
import sys

class ClosedLoopControllerNode(Node):
    def __init__(self):
        super().__init__('closed_loop_controller')
        self.get_logger().info("Closed LoopNode Initialized...")
        self.create_timer(0.1, self.pure_pursuit)

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 1)


        self.sub = self.create_subscription(Pose, "/next_point", self.callback_points, 10)
        self.pub_arrived = self.create_publisher(Bool, "/arrived", 1)

        self.x = None
        self.y = None
        self.q = None

        self.x_d = 0.0
        self.y_d = 0.0

        self.arrived = Bool()
        self.action_finished = False

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.q = msg.theta

        
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

                msg.linear.x = Kv if distance > 0.1 else 0.0
                msg.angular.z = Kw * angle

                self.arrived.data = False
                self.pub_arrived.publish(self.arrived)

            

           if distance < 0.1:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.action_finished = True
                self.arrived.data = True
                self.pub_arrived.publish(self.arrived)

           self.pub.publish(msg)   

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


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    sys.exit()

if __name__ == '__main__':
    main()