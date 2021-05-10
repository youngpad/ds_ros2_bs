import rclpy
from rclpy.node import Node
import getpass


# Import msg
from std_msgs.msg import String
from ds_ros2_msgs.msg import DroneControl
from ds_ros2_msgs.msg import TrajectorySetpoint


# Create setpoint publisher class
class DroneControlNode(Node):

    def __init__(self):
        # Init publisher
        super().__init__('bs_droneControl')

        #Variables
        self.uses = 0
        self.land = False
        self.use = 0
        self.drone_nr = 0

        self.setup()

        # Create publishers
        self.control_publisher = [0]
        self.setpoint_publisher = [0]
        for i in range(1, self.uses+1):
            self.control_publisher.append(self.create_publisher(DroneControl, '/use_0' + str(i) + '/bs_use_control', 10))
            self.setpoint_publisher.append(self.create_publisher(TrajectorySetpoint, '/use_0' + str(i) + '/bs_use_setpoint', 10))

        # Run prompt function
        self.prompt_user()


    def setup(self):
        username = getpass.getuser()
        system_setup = open("/home/" + username + "/system_setup.txt")
        for line in system_setup:
            if "use: " in line:
                space = line.find(" ", 0, len(line))
                self.uses = int(line[space:])

    # Prompt user for commands
    def prompt_user(self):

        # Welcome message
        print("#------------------------#")
        print("DRONESWARM CONTROL CENTER")
        print("#------------------------#")

        # Create DroneControl message instance
        control_msg = DroneControl()
        setpoint_msg = TrajectorySetpoint()

        # Keep prompt going
        while True:
            send_control_input = "not enter"
            send_control_input = input(">>")

            # If input is blank, catch error
            if (send_control_input == ""):
                send_control_input = "blank"

            # Check which use and drone to control
            first_space = send_control_input.find(" ", 0, len(send_control_input))
            if ( (send_control_input[:first_space].isdigit()) and (first_space != -1) ):
                self.use = int(send_control_input[:first_space])
                send_control_input = send_control_input[first_space+1:]

            second_space = send_control_input.find(" ", 0, len(send_control_input))
            if ( (send_control_input[:second_space].isdigit()) and (second_space != -1) ):
                self.drone_nr = int(send_control_input[:second_space])
                control_msg.drone = int(send_control_input[:second_space])
                setpoint_msg.drone = int(send_control_input[:second_space])
                send_control_input = send_control_input[second_space+1:]

            # Invalid input if drone nr or use nr is out of range
            if ((self.use == 999 or self.use in range(1, self.uses+1)) and (self.drone_nr == 999 or self.drone_nr in range(1, 11))):
                None
            else:
                send_control_input = "invalid"

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
        if (self.use == 999):
            for publisher in self.setpoint_publisher:
                if (publisher != 0):
                    publisher.publish(setpoint_msg)
            for publisher in self.control_publisher:
                if (publisher != 0):
                    publisher.publish(control_msg)
        elif (msg == 0):
            self.setpoint_publisher[self.use].publish(setpoint_msg)
            self.control_publisher[self.use].publish(control_msg)
        elif (msg == 1):
            self.setpoint_publisher[self.use].publish(setpoint_msg)
        elif (msg == 2):
            self.control_publisher[self.use].publish(control_msg)

def main(args=None):
    rclpy.init(args=args)

    bs_droneControl = DroneControlNode()

    rclpy.spin(bs_droneControl)

    bs_droneControl.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
