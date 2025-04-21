#!/usr/bin/env python3
import rclpy, time, math
from rclpy.node import Node
from geometry_msgs.msg import Twist



class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.get_logger().info("Turtle Controller node has started")
        self.create_timer(0.1, self.state_machine)
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 1)
        self.t0 = time.time()
        self.v = 0.2
        self.w = 0.2
        self.state = "idle"
        self.turn_comp = False
        self.action_finished = False

    def state_machine(self):

        if self.action_finished:
            self.t0 = time.time()
            self.action_finished = False
            if self.state == "idle": self.state = "state1"
            elif self.state == "state1": self.state = "state2"
            elif self.state == "state2": self.state = "state3"
            elif self.state == "state3": self.state = "state4"
            elif self.state == "state4": self.state ="state5"
            elif self.state == "state5": self.state = "state6"
            elif self.state == "state6": self.state = "state7"
            elif self.state == "state7": self.state = "state8"
            elif self.state == "state8": self.state = "state9"
            elif self.state == "state9": self.state = "state10"
            elif self.state == "state10": self.state = "state3"


        
        # Actions-States
        if self.state == "stop": pass
        if self.state == "idle": self.idle()
        if self.state == "state1": self.advance(7.0710678118654755)
        if self.state == "state2": self.turn(135.0)


        if self.state == "state3": self.advance(10)
        if self.state == "state4": self.turn(90.0)
        if self.state == "state5": self.advance(0.5)
        if self.state == "state6": self.turn(90.0)
        if self.state == "state7": self.advance(10)
        if self.state == "state8": self.turn(-90.0)
        if self.state == "state9": self.advance(0.5)
        if self.state == "state10": self.turn(-90.0)

            


    def advance(self, desired_distance):
        msg = Twist()
        t = time.time() - self.t0
        distance_traveled = self.v * t
        if distance_traveled < desired_distance: 
            msg.linear.x = self.v
            self.pub.publish(msg)
        else:
            self.pub.publish(msg)
            print("Distance condition reached")
            self.action_finished = True

    def idle(self):
        self.turn(225)
    

    def turn(self, desired_angle):
        msg = Twist()
        t = time.time() - self.t0
        angle_traveled = self.w * t * 180 / math.pi  # Convertir a grados
        
        # Definir dirección del giro
        direction = 1.0 if desired_angle > 0 else -1.0
        msg.angular.z = direction * self.w  # Asigna dirección
        
        self.get_logger().info(f"Turning: traveled {angle_traveled:.2f}°, target {desired_angle}°")
        
        if abs(angle_traveled) < abs(desired_angle):  # Comparar valores absolutos
            self.pub.publish(msg)
        else:
            msg.angular.z = 0.0  # Detener giro
            self.pub.publish(msg)
            self.get_logger().info("Angle condition reached")
            self.action_finished = True



def main(args=None):
    rclpy.init(args=args)
    nodeh = TurtleControllerNode()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()