import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

X_CEN = 2760
Y_CEN = 2185

X_MIN = 130
X_MAX = 4095

Y_MIN = 5
Y_MAX = 4095

DEAD = 300

MAX_LIN = 0.12
MAX_ANG = 0.8

PORT = '/dev/ttyACM0'


def axis(v, cen, lo, hi):
    if abs(v - cen) < DEAD:
        return 0.0

    if v > cen:
        return (v - cen) / (hi - cen)

    return (v - cen) / (cen - lo)


class Joy(Node):

    def __init__(self):
        super().__init__('joy_serial')

        self.pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.ser = serial.Serial(
            PORT,
            115200,
            timeout=0.1
        )

        self.strafe = False

        self.create_timer(0.05, self.tick)


    def tick(self):

        try:
            line = self.ser.readline().decode().strip()

            p = line.split(',')

            if len(p) != 3:
                return

            x = int(p[0])
            y = int(p[1])
            btn = int(p[2])

        except:
            return


        # Button switches between turn and strafe mode
        if btn == 1:
            self.strafe = not self.strafe


        ax = axis(
            x,
            X_CEN,
            X_MIN,
            X_MAX
        )

        ay = axis(
            y,
            Y_CEN,
            Y_MIN,
            Y_MAX
        )


        m = Twist()


        # ==================================================
        # TURN MODE
        #
        # UP    -> FORWARD
        # DOWN  -> BACKWARD
        # LEFT  -> LEFT
        # RIGHT -> RIGHT
        # ==================================================

        if not self.strafe:

            m.linear.x = ax * MAX_LIN
            m.linear.y = 0.0
            m.angular.z = ay * MAX_ANG


        # ==================================================
        # STRAFE MODE
        #
        # UP    -> FORWARD
        # DOWN  -> BACKWARD
        # LEFT  -> STRAFE LEFT
        # RIGHT -> STRAFE RIGHT
        # ==================================================

        else:

            m.linear.x = ax * MAX_LIN
            m.linear.y = ay * MAX_LIN
            m.angular.z = 0.0


        self.pub.publish(m)


def main():

    rclpy.init()

    rclpy.spin(Joy())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
