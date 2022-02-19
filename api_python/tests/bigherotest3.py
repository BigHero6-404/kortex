import sys
import os
import time
import threading
from tracemalloc import start
import pyrealsense2 as rs
import cv2 as cv2
import numpy as np
import cgi, cgitb

#from kbhit import KBHit
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2
from kortex_api.autogen.messages import BaseCyclic_pb2
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

# Calling kortex_api module, base_pb2, to make sure the arm is in servoing mode  
def move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
       
    # Move arm to ready position
    print("Moving the arm to a safe position")

    # Setting action type to 
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None

    # walking through action list to search for action labeled "home", once found. The robot will follow under the predetermined home movement
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

# calls base_pd2 module from kortex api to define a command variable as a twist command alias
def move_forward(base, sX, eX):

    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    # divided set object coordinate by .004, to find scale
    # this is used to to define a linear forward movement 
    twist.linear_z = (sX - eX) / 4.975 * -1

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    
    # Let time for twist to be executed
    time.sleep(5)
    base.Stop()
    time.sleep(1)

    return True

#testing for joystick
def switchManualMode(base):
    command = Base_pb2.SwitchControlMapping()
    com = Base_pb2.SWITCH_CONTROL_MAPPING
    val = Base_pb2.ControllerConfigurationMode()
    
    

    return True

#configure and  begin streaming
def startStreaming():
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    while True:
        # This is a blocking call poll_for_frames can be used for async
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)
        

#sweeps workspace from left to right, 
def getDistance(base):
    move_horizontal(base,0.014,-0.197)
    move_vertical(base,0.434,0.137)

    #start_time = time.time()
    #end_time = time.time()
    command = Base_pb2.TwistCommand()
    twist = command.twist
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    minDistance = 10
    yCoordinate = 0
    startWidth = 0
    endWidth = 0
    count = 0
    for x in range(0,60):
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        width = int(depth.get_width())
        height = int(depth.get_height())
        #if not depth: continue
        distance = depth.get_distance(int(width /2),int(height / 2))
       
        print(distance, yCoordinate)
        if(minDistance > distance and distance != 0):
            minDistance = distance
            yCoordinate = 0.321 * (x / 59) - 0.124
         #   if (count == 0):
          #      count += 1
          #      startWidth = yCoordinate
        #elif(distance == minDistance + .09 or distance == minDistance - .9):
          #  endWidth = 0.321 * (x / 59) - 0.124
       # print("Start Width: " + startWidth + "  End Width: " + endWidth)
        twist.linear_x = 0.04
        base.SendTwistCommand(command)
        time.sleep(0.1)
       # print(minDistance, yCoordinate)
    return True, minDistance, yCoordinate

def move_to_object(base, sX, camDis):

    eX = sX + camDis - 0.12
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    # divided set object coordinate by .004, to find scale
    # this is used to to define a linear forward movement 
    twist.linear_z = (sX - eX) / 4.975 * -1

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    
    # Let time for twist to be executed
    time.sleep(5)
    base.Stop()
    time.sleep(1)

    return True, eX

def move_vertical(base, sZ, eZ):

    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0
    
    # Moving the arm down vertically
    twist = command.twist
    twist.linear_y = (sZ - eZ ) / 4.95 * -1
    print(twist.linear_y)

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True

def move_horizontal(base, sY, eY):
    
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0
    
    # Moving the arm down vertically
    twist = command.twist
    twist.linear_x = ( sY - eY ) / 5.175 * -1

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True, eY

class GripperCommandExample:
    def __init__(self, router, router_real_time, proportional_gain = 2.0):

        self.proportional_gain = proportional_gain
        self.router = router
        self.router_real_time = router_real_time

        # Create base client using TCP router
        self.base = BaseClient(self.router)

        ##
         # Create base cyclic client using UDP router.
        self.base_cyclic = BaseCyclicClient(self.router_real_time)
         # Create base cyclic command object.
        self.base_command = BaseCyclic_pb2.Command()
        self.base_command.frame_id = 0
        self.base_command.interconnect.command_id.identifier = 0
        self.base_command.interconnect.gripper_command.command_id.identifier = 0

        # Add motor command to interconnect's cyclic
        self.motorcmd = self.base_command.interconnect.gripper_command.motor_cmd.add()

        # Set gripper's initial position velocity and force
        base_feedback = self.base_cyclic.RefreshFeedback()
        self.motorcmd.position = base_feedback.interconnect.gripper_feedback.motor[0].position
        self.motorcmd.velocity = 0
        self.motorcmd.force = 100

        

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
        position = 1
        finger.value = position
        self.base.SendGripperCommand(gripper_command)
        time.sleep(1)

        
            


def main():
    #endX = float(endX)
    #endY = float(endY)
    #endZ = float(endZ)
    #print(endX, endY, endZ)
    #homeX, homeY, homeZ = 0.575, 0.014, 0.434
    #startX, startY, startZ = [float(s) for s in input("Enter the coordiantes your object is currently located at (x, y, z): ").split()]
    #form = cgi.FieldStorage()
    #endX = form.getvalue('x')
    #endY = form.getvalue('y')
    #endZ = form.getvalue('z')
    endX, endY, endZ = [float(s) for s in input("Enter the coordinates you would like to drop your object off at (x, y, z): ").split()]
    #endX = 0.536
    #endY = -0.142
    #endZ = 0.14
    
    # Import the utilities helper module
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        with utilities.DeviceConnection.createUdpConnection(args) as router_real_time:

            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)
            min = 0
            y = 0
            pickUpY = 0
            pickUpX = 0
            # Moving to home position

            
            success = True
            success &= move_to_home_position(base)
            success &= move_to_home_position(base)
            success &= switchManualMode(base)
            #start streaming
            #p1 = Process(target = startStreaming)
           # p1.start()
           # p2, min, y = Process(target = getDistance(base))
            success, min, y = getDistance(base)

            #Moving to object after detection
            success, pickUpY = move_horizontal(base, 0.18, y)
            success, pickUpX = move_to_object(base, 0.572, min)
            example = GripperCommandExample(router, router_real_time)
            example.SendGripperCommands()
            
            success &= move_vertical(base,-0.434, -0.137)
            success = move_horizontal(base, pickUpY, endY)
            success = move_forward(base, pickUpX, endX)
            success &= move_vertical(base,0.434, 0.137)
            example.ResetGrippers()
            success &= move_vertical(base,-0.434,-0.137)
            success &= move_to_home_position(base)

            #Move object to end position and reset gripper
            #success &= move_horizontal(base, )

            return 0 if success else 1
if __name__ == "__main__":
    exit(main())