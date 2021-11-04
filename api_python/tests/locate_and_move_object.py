import sys
import os
import time
import threading

#from kbhit import KBHit
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check

def move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
       
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def move_forward(base, sZ, eZ):

    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_z = (sZ - eZ) / 4.975 * -1

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    
    # Let time for twist to be executed
    time.sleep(5)
    base.Stop()
    time.sleep(1)

    return True


def move_vertical(base, sY, eY):
    
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0
    
    # Moving the arm down vertically
    twist = command.twist
    twist.linear_y = (sY - eY ) / 4.95 * -1
    print(twist.linear_y)

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True
# list of remaining methods:

# vertically, horizontally, vertically, release grippers (use resetGrippers)
# for the second-to-last definition, call out twist.linear_x

def move_horizontal(base, sX, eX):
    
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0
    
    # Moving the arm down vertically
    twist = command.twist
    twist.linear_x = ( sX - eX ) / 5.175 * -1

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True

class GripperCommandExample:
    def __init__(self, router, proportional_gain = 2.0):

        self.proportional_gain = proportional_gain
        self.router = router

        # Create base client using TCP router
        self.base = BaseClient(self.router)

    def ResetGrippers(self):

        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Resetting gripper to standard position
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 0
        finger.value = position
        self.base.SendGripperCommand(gripper_command)
        time.sleep(1)

    def SendGripperCommands(self):

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        print("Performing gripper test in position...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 0.35
        finger.value = position
        self.base.SendGripperCommand(gripper_command)
        time.sleep(1)


def main():

    homeX, homeY, homeZ = 0.575, 0.014, 0.434
    startX, startY, startZ = [float(s) for s in input("Enter the coordiantes your object is currently located at (x, y, z): ").split()]
    endX, endY, endZ = [float(s) for s in input("Enter the coordinates you would like to drop your object off at (x, y, z): ").split()]

    # Import the utilities helper module
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
       

        # Moving to home position
        success = True
        success &= move_to_home_position(base)
        success &= move_to_home_position(base)

        # reset grippers
        example = GripperCommandExample(router)
        example.ResetGrippers()

        args = utilities.parseConnectionArguments()


       

        success &= move_vertical(base, homeZ, startZ)
        success &= move_horizontal(base, homeY, startY)
        success &= move_forward(base, homeX, startX)
    
        example = GripperCommandExample(router)
        example.SendGripperCommands()

        success &= move_vertical(base, startZ, homeZ)
        success &= move_horizontal(base, startY, endY)
        success &= move_forward(base, startX, endX)
        success &= move_vertical(base, homeZ, startZ)

        example.ResetGrippers()
        success &= move_vertical(base, startZ, homeZ)
        success = True
        success &= move_to_home_position(base)
       # success &= move_to_home_position(base)

        return 0 if success else 1

if __name__ == "__main__":
    exit(main())
