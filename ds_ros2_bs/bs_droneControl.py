import rclpy
from rclpy.node import Node

# Import msg
from std_msgs.msg import String
from ds_ros2_msgs.msg import DroneControl
from ds_ros2_msgs.msg import TrajectorySetpoint


# Create setpoint publisher class
class DroneControlNode(Node):

    def __init__(self):
        # Init publisher
        super().__init__('bs_droneControl')

        # Create publishers
        self.control_publisher_01 = self.create_publisher(DroneControl, 'bs_use_01_control', 10)
        self.setpoint_publisher_01 = self.create_publisher(TrajectorySetpoint, 'bs_use_01_setpoint', 10)
        self.control_publisher_02 = self.create_publisher(DroneControl, 'bs_use_02_control', 10)
        self.setpoint_publisher_02 = self.create_publisher(TrajectorySetpoint, 'bs_use_02_setpoint', 10)

        # Variables
        self.land = False
        self.use_nr = 0
        self.nr_of_use = 0

        # Run prompt function
        self.prompt_user()


    # Prompt user for commands
    def prompt_user(self):

        # Create DroneControl message instance
        control_msg = DroneControl()
        setpoint_msg = TrajectorySetpoint()

        # Welcome message
        print("#------------------------#")
        print("DRONESWARM CONTROL CENTER")
        print("#------------------------#")

        # Keep prompt going
        while True:

            send_control_input = "not enter"
            send_control_input = input(">>")

            # If input is blank, catch error
            if (send_control_input == ""):
                send_control_input = "blank"

            # Check which use and drone to control
            first_space = send_control_input.find(" ", 0, len(send_control_input))
            second_space = send_control_input.find(" ", first_space, len(send_control_input))

            if ( (send_control_input[:first_space].isdigit()) and (first_space != -1) ):
                self.use_nr = int(send_control_input[:first_space])
                send_control_input = send_control_input[first_space+1:]

            if ( (send_control_input[:second_space].isdigit()) and (second_space != -1) ):
                control_msg.drone = int(send_control_input[:second_space])
                setpoint_msg.drone = int(send_control_input[:second_space])
                send_control_input = send_control_input[second_space+1:]

            # Check command input
            if (send_control_input.upper() == "DISARM"):

                # Disarm
                print("Disarm command sent..")
                control_msg.arm = False
                self.publish_(0, control_msg, setpoint_msg)

            elif (send_control_input.upper() == "ARM"):

                # Disarm before arm
                control_msg.arm = False
                control_msg.land = False
                self.publish_(0, control_msg, setpoint_msg)

                # Arm and offboard control true
                print("Arm command sent..")
                control_msg.arm = True
                self.publish_(0, control_msg, setpoint_msg)

            elif (send_control_input.upper() == "LAUNCH"):
                control_msg.launch = True
                self.publish_(0, control_msg, setpoint_msg)
                print("Launch command sent...")

            elif (send_control_input.upper() == "LAND"):
                control_msg.land = True
                self.publish_(0, control_msg, setpoint_msg)
                print("Land command sent...")

            elif (send_control_input.upper() == "TURN PX ON"):
                control_msg.switch_px = True
                self.publish_(0, control_msg, setpoint_msg)
                print("Switched on pixhawk...")

            elif (send_control_input.upper() == "TURN PX OFF"):
                control_msg.switch_px = False
                self.publish_(0, control_msg, setpoint_msg)
                print("Switched off pixhawk...")

            elif (send_control_input.upper()[0] == "X"):
                print("Setpoint x sent...")
                setpoint_msg.x = float(send_control_input[1:])
                self.publish_(0, control_msg, setpoint_msg)

            elif (send_control_input.upper()[0] == "Y"):
                print("Setpoint y sent...")
                setpoint_msg.y = float(send_control_input[1:])
                self.publish_(0, control_msg, setpoint_msg)

            elif (send_control_input.upper()[0] == "Z"):
                print("Setpoint z sent...")
                setpoint_msg.z = float(send_control_input[1:])
                self.publish_(0, control_msg, setpoint_msg)

            elif (send_control_input.upper()[0:2] == "VZ"):
                print("Setpoint vz sent...")
                setpoint_msg.vz = float(send_control_input[2:])
                self.publish_(0, control_msg, setpoint_msg)

            elif (send_control_input.upper() == "VELOCITY"):
                setpoint_msg.x = float("NaN")
                setpoint_msg.y = float("NaN")
                setpoint_msg.z = float("NaN")
                setpoint_msg.yaw = float("NaN")
                setpoint_msg.yawspeed = float("NaN")
                setpoint_msg.vx = 0.0
                setpoint_msg.vy = 0.0
                setpoint_msg.vz = 0.0
                setpoint_msg.acceleration = [float("NaN"), float("NaN"), float("NaN")]
                setpoint_msg.jerk = [float("NaN"), float("NaN"), float("NaN")]
                setpoint_msg.thrust = [float("NaN"), float("NaN"), float("NaN")]

                control_msg.arm = False
                control_msg.launch = False

                print("Ready for velocity setpoints..")

                self.publish_(0, control_msg, setpoint_msg)


            elif (send_control_input.upper() == "RESET"):
                setpoint_msg.x = 0.0
                setpoint_msg.y = 0.0
                setpoint_msg.z = 0.0
                setpoint_msg.yaw = 0.0
                setpoint_msg.yawspeed = 0.0
                setpoint_msg.vx = 0.0
                setpoint_msg.vy = 0.0
                setpoint_msg.vz = 0.0
                setpoint_msg.acceleration = [0.0, 0.0, 0.0]
                setpoint_msg.jerk = [0.0, 0.0, 0.0]
                setpoint_msg.thrust = [0.0, 0.0, 0.0]

                control_msg.arm = False
                control_msg.launch = False

                print("All reset..")

                self.publish_(0, control_msg, setpoint_msg)

            else:
                print("invalid command..")

    def publish_(self, msg, control_msg, setpoint_msg):

        if (self.use_nr == 1 and msg == 0):
            self.setpoint_publisher_01.publish(setpoint_msg)
            self.control_publisher_01.publish(control_msg)
            print("Published all to USE 1")
        elif (self.use_nr == 1 and msg == 1):
            self.setpoint_publisher_01.publish(setpoint_msg)
            print("Published setpoint to USE 1")
        elif (self.use_nr == 1 and msg == 2):
            self.control_publisher_01.publish(control_msg)
            print("Published control to USE 1")
        elif (self.use_nr == 2 and msg == 0):
            self.setpoint_publisher_02.publish(setpoint_msg)
            self.control_publisher_02.publish(control_msg)
            print("Published all to USE 2")
        elif (self.use_nr == 2 and msg == 1):
            self.setpoint_publisher_02.publish(setpoint_msg)
            print("Published setpoint to USE 2")
        elif (self.use_nr == 2 and msg == 2):
            self.control_publisher_02.publish(control_msg)
            print("Published control to USE 2")
        elif (self.use_nr == 999 and msg == 0):
            self.setpoint_publisher_01.publish(setpoint_msg)
            self.control_publisher_01.publish(control_msg)
            self.setpoint_publisher_02.publish(setpoint_msg)
            self.control_publisher_02.publish(control_msg)
        elif (self.use_nr == 999 and msg == 1):
            self.setpoint_publisher_01.publish(setpoint_msg)
            self.setpoint_publisher_02.publish(setpoint_msg)
        elif (self.use_nr == 999 and msg == 2):
            self.control_publisher_01.publish(control_msg)
            self.control_publisher_02.publish(control_msg)







def main(args=None):
    rclpy.init(args=args)

    bs_droneControl = DroneControlNode()

    rclpy.spin(bs_droneControl)

    bs_droneControl.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
