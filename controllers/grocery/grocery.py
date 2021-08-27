"""grocery controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import sys
import tempfile
import requests 
from threading import Thread




# Verbose mode
vrb = False

#Initialization
print("=== Initializing Grocery Shopper...")

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#Consts
MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12
LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)
CAM_POS = (-0.013797,0.137,0.326805)
CAM_WIDTH = 240
CAM_HEIGHT = 135

gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

goals = [(2,0.0),(2.4,-1),(2.4,-2.5), (1.2,-3), (0.95,-4.16),(0.95,-5.4)]
cur_index = 0

# start of necessary code
stopping_threshold = .4

# Creating dictionary for objects aisles
# Aisles go from 1 to 4 from left->right when looking at back staircase
object_aisle_dict = {"beer": 1,
                     "cereal": 1,
                     "orange": 2,
                     "apple": 2,
                     "jam": 2,
                     "rubber_duck": 3,
                     "honey": 3,
                     "biscuit": 4
                     }

aisle_start_points = [[-9.8,-1.7],[-9.8,2.1],[-9.8,5.9],[-9.8,10.1]]
aisle_end_points = [[7,-1.7],[7,2.1],[7,5.9],[7,10.1]]

# target_item = "cereal"

#Global Vars
notRecognized = True
ikResults = [100,0,0,0,0,0,0,0,0,0,0]
ikTargetIK = None
ikTargetDown = None
ikTargetUp = None
robotHeightDelta = 0.07
state = "START"

reached_aisle_start = False
reached_aisle_end = False
aisle_search_complete = False
desired_rotation = 0 # 0 for starting aisle at going down, one for at end of aisle and going back up
aisle_targeting_state = 1
LOOKING_SPEED = MAX_SPEED/10
active_max_speed = MAX_SPEED
new_counter = 0
ramping_counter = 0
ramp_active = False
temp = 0
temp_counter = False
dist_gain = 1
theta_gain = 1
initial_rotation = True
left_spin_complete = False
previous_error = 100

# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
target_item_list = ["orange", "beer"]
# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

target_item_index = 0
target_item = target_item_list[target_item_index]
num_target_items = len(target_item_list)
home_coords = [-17, 5]
increase_index_next = False
done_shopping = False
current_goal = None
previous_orientation = 0
previous_coords = [0,0,0]

received_shopping_list = False
retry_count = 15

# Web Stuff
def checker_thread():
    global received_shopping_list, target_item_list, target_item, num_target_items, retry_count
    try:
        requests.get("http://localhost/await")
    except:
        print("No valid webserver found, grabbing orange and beer")
        retry_count = -5
        received_shopping_list = True
        
    while not received_shopping_list and retry_count > 0:
        retry_count -= 1
        r = requests.get("http://localhost/await")
        print("Did not receive shopping list, {} attempts left".format(retry_count))
        if r.json():
            r = requests.get("http://localhost/list")
            target_item_list = r.json()
            target_item = target_item_list[0]
            received_shopping_list = True
            num_target_items = len(target_item_list)
    
    if(retry_count > -5):
        print("Did not receive shopping list in time, grabbing orange and beer")
    received_shopping_list = True

checkThread = Thread(target=checker_thread)
checkThread.start()

#Enable Sensors
robot_parts=[]

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")


# Set initial values for motors
robot.getDevice("wheel_left_joint").setPosition(math.inf)
robot.getDevice("wheel_left_joint").setVelocity(0)
robot.getDevice("wheel_right_joint").setPosition(math.inf)
robot.getDevice("wheel_right_joint").setVelocity(0)
robot.getDevice("head_1_joint").getPositionSensor().enable(timestep)
robot.getDevice("head_1_joint").setVelocity(0.5)
robot.getDevice("torso_lift_joint").setPosition(0.15)
robot.getDevice("torso_lift_joint").getPositionSensor().enable(timestep)
# robot.getDevice("gripper_right_finger_joint").getPositionSensor().enable(timestep)
robot.getDevice("gripper_right_finger_joint").setPosition(0.045)
robot.getDevice("gripper_right_finger_joint").setVelocity(0.05)
# robot.getDevice("gripper_left_finger_joint").getPositionSensor().enable(timestep)
robot.getDevice("gripper_left_finger_joint").setPosition(0.045)
robot.getDevice("gripper_left_finger_joint").setVelocity(0.05)

# Enable Camera
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# ------------------------------------------------------------------


# Create the IKPY Chain
my_chain = Chain.from_urdf_file("tiago_urdf.urdf", last_link_vector=[0.004, 0,-0.1741], base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_11367_joint", "TIAGo front arm_11367"])
print("=== Created IKPY chain")
if(vrb):
    print(my_chain.links)

# ------------------------------------------------------------------


print("=== Disabling Links...")
# Disable any links we cannot control
for link_id in range(len(my_chain.links)):

    # This is the actual link object
    link = my_chain.links[link_id]
    
    # I've disabled "torso_lift_joint" manually as it can cause
    # the TIAGO to become unstable.
    if link.name not in part_names or  link.name =="torso_lift_joint":
        if(vrb):
            print("Disabling {}".format(link.name))
        my_chain.active_links_mask[link_id] = False


# ------------------------------------------------------------------

print("=== Initializing Motors...")
# Initialize the arm motors and encoders.
motors = []
for link in my_chain.links:
    if link.name in part_names and link.name != "torso_lift_joint":
        if(vrb):
            print("Initializing {}...".format(link.name))

        motor = robot.getDevice(link.name)

        # Make sure to account for any motors that
        # require a different maximum velocity!
        if link.name == "torso_lift_joint":
            motor.setVelocity(0.07)
        else:
            motor.setVelocity(1)
            
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timestep)
        motors.append(motor)


# ------------------------------------------------------------------
# Helper Functions

def rotate_y(x,y,z,theta):
    new_x = x*np.cos(theta) + y*np.sin(theta)
    new_z = z
    new_y = y*-np.sin(theta) + x*np.cos(theta)
    return [-new_x, new_y, new_z]

def lookForTarget(recognized_objects):
    if len(recognized_objects) > 0:

        for item in recognized_objects:
            if target_item in str(item.get_model()):

                target = recognized_objects[0].get_position()
                dist = abs(target[2])

                if dist < 5:
                    return True

def checkArmAtPosition(ikResults, cutoff=0.00005):
    '''Checks if arm at position, given ikResults'''
    
    # Get the initial position of the motors
    initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]

    # Calculate the arm
    arm_error = 0
    for item in range(14):
        arm_error += (initial_position[item] - ikResults[item])**2
    arm_error = math.sqrt(arm_error)

    if arm_error < cutoff:
        if vrb:
            print("Arm at position.")
        return True
    return False

def moveArmToTarget(ikResults):
    '''Moves arm given ikResults'''
    # Set the robot motors
    for res in range(len(ikResults)):
        if my_chain.links[res].name in part_names:
            # This code was used to wait for the trunk, but now unnecessary.
            # if abs(initial_position[2]-ikResults[2]) < 0.1 or res == 2:
            robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
            if vrb:
                print("Setting {} to {}".format(my_chain.links[res].name, ikResults[res]))

def calculateIk(offset_target,  orient=True, orientation_mode="Y", target_orientation=[0,0,1]):
    '''
    This will calculate the iK given a target in robot coords
    Parameters
    ----------
    param offset_target: a vector specifying the target position of the end effector
    param orient: whether or not to orient, default True
    param orientation_mode: either "X", "Y", or "Z", default "Y"
    param target_orientation: the target orientation vector, default [0,0,1]

    Returns
    ----------
    rtype: bool
        returns: whether or not the arm is at the target
    '''

    # Get the initial position of the motors
    initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
    
    # Calculate IK
    ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position,  target_orientation = [0,0,1], orientation_mode="Y")

    # Use FK to calculate squared_distance error
    position = my_chain.forward_kinematics(ikResults)

    # This is not currently used other than as a debug measure...
    squared_distance = math.sqrt((position[0, 3] - offset_target[0])**2 + (position[1, 3] - offset_target[1])**2 + (position[2, 3] - offset_target[2])**2)
    print("IK calculated with error - {}".format(squared_distance))

    # Reset the ikTarget (deprec)
    # ikTarget = offset_target
    
    return ikResults

    # Legacy code for visualizing
        # import matplotlib.pyplot
        # from mpl_toolkits.mplot3d import Axes3D
        # ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

        # my_chain.plot(ikResults, ax, target=ikTarget)
        # matplotlib.pyplot.show()
        
def getTargetFromObject(recognized_objects):
    ''' Gets a target vector from a list of recognized objects '''

    # Get the first valid target
    target = recognized_objects[0].get_position()

    # Convert camera coordinates to IK/Robot coordinates
    # offset_target = [-(target[2])+0.22, -target[0]+0.08, (target[1])+0.97+0.2]
    offset_target = [-(target[2])+0.22, -target[0]+0.06, (target[1])+0.97+0.2]

    return offset_target

def reachArm(target, previous_target, ikResults, cutoff=0.00005):
    '''
    This code is used to reach the arm over an object and pick it up.
    '''

    # Calculate the error using the ikTarget
    error = 0
    ikTargetCopy = previous_target

    # Make sure ikTarget is defined
    if previous_target is None:
        error = 100
    else:
        for item in range(3):
            error += (target[item] - previous_target[item])**2
        error = math.sqrt(error)

    
    # If error greater than margin
    if error > 0.05:
        print("Recalculating IK, error too high {}...".format(error))
        ikResults = calculateIk(target)
        ikTargetCopy = target
        moveArmToTarget(ikResults)

    # Exit Condition
    if checkArmAtPosition(ikResults, cutoff=cutoff):
        if vrb:
            print("NOW SWIPING")
        return [True, ikTargetCopy, ikResults]
    else:
        if vrb:
            print("ARM NOT AT POSITION")

    # Return ikResults
    return [False, ikTargetCopy, ikResults]

def closeGrip():
    robot.getDevice("gripper_right_finger_joint").setPosition(0.0)
    robot.getDevice("gripper_left_finger_joint").setPosition(0.0)

    # r_error = abs(robot.getDevice("gripper_right_finger_joint").getPositionSensor().getValue() - 0.01)
    # l_error = abs(robot.getDevice("gripper_left_finger_joint").getPositionSensor().getValue() - 0.01)
    
    # print("ERRORS")
    # print(r_error)
    # print(l_error)

    # if r_error+l_error > 0.0001:
    #     return False
    # else:
    #     return True

def openGrip():
    robot.getDevice("gripper_right_finger_joint").setPosition(0.045)
    robot.getDevice("gripper_left_finger_joint").setPosition(0.045)

    # r_error = abs(robot.getDevice("gripper_right_finger_joint").getPositionSensor().getValue() - 0.045)
    # l_error = abs(robot.getDevice("gripper_left_finger_joint").getPositionSensor().getValue() - 0.045)

    # if r_error+l_error > 0.0001:
    #     return False
    # else:
    #     return True

def delay(time):
    '''delays by time (seconds)'''
    now_time = robot.getTime()
    target_time = robot.getTime()+time
    while target_time-now_time>0:
        now_time = robot.getTime()
        if vrb:
            print("{} seconds remaining...".format(target_time-now_time))

        #Still move the robot
        robot.getDevice("wheel_left_joint").setVelocity(vL)
        robot.getDevice("wheel_right_joint").setVelocity(vR)

        robot.step(timestep)

# ------------------------------------------------------------------


# Main Loop
while robot.step(timestep) != -1:
    

    # Setup Globals
    vL = 0
    vR = 0
    all_recognized_objects = camera.getRecognitionObjects()
    recognized_objects = []
    for item in all_recognized_objects:
        if target_item in str(item.get_model()):
            recognized_objects.append(item)
    

    # IK State
    if state == "IK":
        # State Function
        [state, ikTargetIK, ikResults] = reachArm(getTargetFromObject(recognized_objects), ikTargetIK, ikResults)
        #State Transition
        if state == True:
            ikResults = [100,0,0,0,0,0,0,0,0,0,0,0,0,0]
            print("=== LOWERING")
            state = "LOWERING"
        else:
            state = "IK"

    # Grabbing
    if state == "LOWERING":
        # Move arm down
        lResult = False
        if target_item == "biscuit":
            [lResult, ikTargetDown, ikResults] = reachArm([ikTargetIK[0], ikTargetIK[1], ikTargetIK[2]-0.22] , ikTargetDown, ikResults)
        elif target_item == "rubber_duck":
            [lResult, ikTargetDown, ikResults] = reachArm([ikTargetIK[0], ikTargetIK[1], ikTargetIK[2]-0.195] , ikTargetDown, ikResults) 
        else:
            [lResult, ikTargetDown, ikResults] = reachArm([ikTargetIK[0], ikTargetIK[1], ikTargetIK[2]-0.19] , ikTargetDown, ikResults)

        delay(0.325)
        closeGrip()

        # State Transition
        if lResult == True:
            ikResults = [100,0,0,0,0,0,0,0,0,0,0,0,0,0]
            print("=== GRABBING")
            state = "GRABBING"
        else:
            state = "LOWERING"
    
    # Grabbing
    if state == "GRABBING":
        closeGrip()
       
        delay(0.5)
        print("=== LIFTING")
        state = "LIFTING"

    if state == "LIFTING":
        # Move arm back up
        uResult = False
        [uResult, ikTargetUp, ikResults] = reachArm([ikTargetDown[0], ikTargetDown[1], ikTargetDown[2]+0.22], ikTargetUp, ikResults)

        # State Transition
        if uResult == True:
            print("=== RESET POSITION")
            ikTargetUp = None
            ikTargetDown = None
            ikTargetIK = None
            ikResults = [100,0,0,0,0,0,0,0,0,0,0,0,0,0]
            state = "RESET"
        else:
            state = "LIFTING"

    if state == "RETURNING":
        # Move arm back towards self
        ikResults = [0,0,0,0,0.07,0,-1.5,2.29,-1.8,1.1,-1.4,0,0,0]
        # ikResults = [0,0,0,0,0.07,1.02,-1.5,2.29,-1.8,1.1,-1.4,0,0,0]
        moveArmToTarget(ikResults)        
        
        # State Transition
        if checkArmAtPosition(ikResults, 0.00005):
            print("=== DROPPING")
            state = "DROPPING"
        else:
            state = "RETURNING"

    if state == "DROPPING":
        openGrip()
       
        delay(0.5)
        
        print("=== ROTATING TO DESIRED")
        # state = "STOP"
        state = "ROTATE TO DESIRED"
        desired_rotation = math.pi/2
        reached_aisle_start = False
        increase_index_next = True
    
    if (state == "RESET"):
        vL = -3
        vR = -3
        
        delay(2)
        print("=== RETURNING")
        state = "RETURNING"

        # error = math.sqrt((gps.getValues()[0]-previous_coords[0])**2 + (gps.getValues()[1]-previous_coords[1])**2 + (gps.getValues()[2]-previous_coords[2])**2)
        # print(error)

        # if error < previous_error:
        #     previous_error = 100
        #     print("=== RETURNING")
        #     state = "RETURNING"
        #     # print("=== RESETTING ORIENTATION")
        #     # state = "RESET ORIENTATION"
        # else:
        #     previous_error = error

    if (state == "RESET ORIENTATION"):
        n = compass.getValues()
        rad = ((math.atan2(n[0], n[2]))-1.5708)

        diff = rad - previous_orientation

        if diff > -0.1 and diff < 0.1:
            print("=== RETURNING")
            state = "RETURNING" 
        elif diff < 0:
            vL = 1
            vR = -1
        elif diff > 0:
            vL = -1
            vR = 1


    # Traveling
    if state == "TRAVELING":

        if(vrb):
            print("******************************************************")
            print("vL measured: %f" % robot.getDevice("wheel_left_joint").getVelocity())
            print("vR measured: %f" % robot.getDevice("wheel_right_joint").getVelocity())
        # starting movement code
        # getting pose from gps -> from lab 5
        pose_y = gps.getValues()[2]
        pose_x = gps.getValues()[0]
        n = compass.getValues()
        rad = -((math.atan2(n[0], n[2])) - 1.5708)
        pose_theta = rad



        #test_pose = compass.getValues()[1] - math.pi/2
        #pose_theta=test_pose
        #print("test pose: %f" % test_pose)
        # if temp_counter:
            # temp = temp + 1
            # if temp > 50:
                # print("hold finished")
                # temp_counter = False

        # temp state -1
    # elif aisle_targeting_state == -1:
        # wheel_speed_temp = MAX_SPEED
        # robot_parts[MOTOR_LEFT].setVelocity(wheel_speed_temp)
        # robot_parts[MOTOR_RIGHT].setVelocity(wheel_speed_temp)
        # vL = wheel_speed_temp
        # vR = wheel_speed_temp
        if current_goal is None:
            reached_aisle_start = False

        if reached_aisle_start == False:
            if(vrb):
                print("going to start of aisle")
            # if not yet at start of aisle -> sets start as desired
            target_aisle = object_aisle_dict[target_item]
            current_goal = aisle_start_points[target_aisle-1]
            active_max_speed = MAX_SPEED
            dist_gain = .6
            theta_gain = 1.6
        elif reached_aisle_end == False:
            # print("going down aisle")
            # if robot has been to start but not yet at the end -> sets end as desired

            # if the robot reached aisle start after finding aisle -> goes to next 
            if increase_index_next:
                if target_item_index == (num_target_items-1):
                    state = "TRAVELING"
                    done_shopping = True
                    increase_index_next = False
                    reached_aisle_start = False
                else:
                    state = "ROTATE TO DESIRED"
                    current_aisle = object_aisle_dict[target_item]
                    next_item = target_item_list[target_item_index+1]
                    next_aisle = object_aisle_dict[next_item]
                    if next_aisle>current_aisle:
                        desired_rotation = math.pi
                    elif next_aisle<current_aisle:
                        desired_rotation = 0
                    else:
                        desired_rotation = 3*math.pi/2
                    vL = 0
                    vR = 0
                    increase_index_next = False
                    target_item_index+=1
                    target_item = target_item_list[target_item_index]
                    reached_aisle_start = False
                    current_goal = None

                # IF ALL ITEMS ARE FOUND -> GO HOME
                

            else:
                # if it is going to search for 
                vL = 1
                vR = 1
                delay(3)
                print("=== TURNING HEAD LEFT")
                state = "TURN HEAD LEFT"
            
            # target_aisle = object_aisle_dict[target_item]
            # current_goal = aisle_end_points[target_aisle - 1]
            # active_max_speed = MAX_SPEED
            # ramp_active = True
            # ramping_counter = ramping_counter + 1
            # dist_gain = 1
            # theta_gain = 1.3
        # elif reached_aisle_start and aisle_search_complete is False:
            # print("returning back to start of aisle")
            # if gone from top to bottom of aisle, going back to top
            # target_aisle = object_aisle_dict[target_item]
            # current_goal = aisle_start_points[target_aisle - 1]
            # active_max_speed = MAX_SPEED
            # ramp_active = False
            # ramping_counter = ramping_counter + 1
            # dist_gain = 1
            # theta_gain = 1.3

        else:
            if(vrb):
                print("no desired position")
            current_goal = None

        # OVERRIDES CURRENT GOAL WHEN DONE SHOPPING
        if done_shopping:
            current_goal = home_coords

        if current_goal is not None:

            # CALCULATING VARIOUS ERRORS BASED OFF GOAL POSITION
            error_x = pose_x - current_goal[0]
            error_y = pose_y - current_goal[1]
            error_dist = math.sqrt(math.pow(error_x, 2) + math.pow(error_y, 2))

            atan = -math.atan2(current_goal[1] - pose_y, current_goal[0] - pose_x) + math.pi

            rpose_theta = pose_theta
            if rpose_theta > math.pi:
                rpose_theta -= 2 * math.pi

            rpose_theta += math.pi
            error_theta = atan - rpose_theta - math.pi/2

            if -2*math.pi < error_theta < -1*math.pi:
                error_theta += 2 * math.pi
            if math.pi < error_theta < 2*math.pi:
                error_theta -= 2 * math.pi

            x_dot = error_dist * dist_gain
            theta_dot = (error_theta) * theta_gain
            if(vrb):
                print("ERROR: X: %f Y: %f DIST: %f THETA: %f" % (error_x, error_y, error_dist, error_theta))
        else:
            if(vrb):
                print("not running travel code this time")
            x_dot = 0
            theta_dot = 0


        # if initial_rotation:
        #     desired_rotation = pose_theta + error_theta
        #     aisle_targeting_state = 2
        #     initial_rotation = False

        # STOPPING CRITERIA -> moves to state 2
        if error_dist < stopping_threshold:
            # initial_rotation = True
            # new_counter = 0
            print("\n\n@@@@@@@ TRAVELLING STOP CRITERIA MET @@@@@@@\n\n")
            x_dot = 0
            theta_dot = 0
            if reached_aisle_start == False:
                if increase_index_next is False and current_goal is not None:
                    desired_rotation = 3*math.pi/2
                    state = "ROTATE TO DESIRED"
                    reached_aisle_start = True
                else:
                    reached_aisle_start = True

            if done_shopping:
                state = "STOP"
                if(vrb):
                    print("SHOPPING COMPLETE.")
                
            # elif reached_aisle_end == False:
                # desired_rotation = math.pi/2
                # aisle_targeting_state = 2
                # reached_aisle_end = True
            # elif reached_aisle_start and reached_aisle_end and aisle_search_complete is False:
                # aisle_search_complete = True
                # aisle_targeting_state = 0


        #STEP 3: Compute wheelspeeds

        # MAX_SPEED [rad/s] * r [m] = MAX_SPEED_MS [m/s]
        r = MAX_SPEED_MS/MAX_SPEED

        k_prop = .2

        vL = k_prop*(2*x_dot-theta_dot*AXLE_LENGTH)/(2*r)
        vR = k_prop*(2*x_dot+theta_dot*AXLE_LENGTH)/(2*r)
        if(vrb):
            print("original* vL: %f vR: %f" % (vL, vR))

        #STEP 4: Normalize wheelspeed
        ratio = math.inf

        vL_sign = 1
        if vL < 0:
            vL_sign = -1
        vR_sign = 1
        if vR < 0:
            vR_sign = -1

        vL = abs(vL)
        vR = abs(vR)

        if vL > active_max_speed or vR > active_max_speed:
            if abs(vL) > abs(vR):
                reducer = vL/active_max_speed
                vL = active_max_speed
                vR = vR/reducer
            elif abs(vL) < abs(vR):
                reducer = vR/active_max_speed
                vR = active_max_speed
                vL = vL/reducer
            ratio = vR/vL
            # if .99 < ratio < 1.01:
            #     vL = active_max_speed
            #     vR = active_max_speed

        vL = vL*vL_sign
        vR = vR*vR_sign

        if abs(vR) < 1 and abs(vL) < 1:
            vR = vR*3
            vL = vL*3
        elif abs(vR) < 2 and abs(vL) < 2:
            vR = vR*2
            vL = vL*2

        if ramp_active:
            ramping_factor = ramping_counter / 2e3
            if ramping_factor > 1 or ramping_counter > 100:
                ramping_factor = 1
            vL = vL * ramping_factor
            vR = vR * ramping_factor

        if dist_gain == 1:
            vR = 7
            vL = 7

        
        if(vrb):
            print("commanded* vL: %f vR: %f" % (vL, vR))

            print("X: %f Y: %f Theta: %f\n" % (pose_x, pose_y, pose_theta))  # Comment out this if you want


    if state == "ROTATE TO DESIRED":
        if(vrb):
            print("original theta: %f" % pose_theta)

        n = compass.getValues()
        rad = -((math.atan2(n[0], n[2])) - 1.5708)
        pose_theta = rad

        # adjusting theta to ensure correct theta error
        if pose_theta > 2*math.pi:
            # print("pose theta changed > 2pi")
            pose_theta = pose_theta - 2*math.pi
        if pose_theta < 0:
            # print("pose theta changed, < 0")
            pose_theta = 2*math.pi + pose_theta
        if(vrb):
            print("rotational state pose theta: %f" % pose_theta)
            print("desired rotation: %f" % desired_rotation)
        error_theta = desired_rotation - pose_theta

        # desired orientation is zero or pi (down or up aisle)
        rotational_threshold = .02
        if abs(error_theta) < rotational_threshold or abs(error_theta) > (2*math.pi-rotational_threshold):
            
            vL = 0
            vR = 0
            
            state = "TRAVELING"
            
            #temp_counter = False
            #ramping_counter = 0
        else:
            # Right jitter happens in THIS code.
            # - Only happens when turning right

            # spinning the robot until the orientation is correct
            #new_counter = 0
            rotation_sign = -1
            # if the robot crosses the 2pi line, spin the other way
            # if abs(error_theta-math.pi) < math.pi:
            #     rotation_sign = 1
                
            turning_speed = 2

            if abs(error_theta) < .2 or abs(error_theta) > (2*math.pi-.2):
                turning_speed = .1

            turning_speed = turning_speed*rotation_sign

            vL = turning_speed
            vR = -1*turning_speed
            #robot_parts[MOTOR_LEFT].setVelocity(turning_speed)
            #robot_parts[MOTOR_RIGHT].setVelocity(-1*turning_speed)
            if(vrb):
                print("error theta: %f\n" % error_theta)

        # DELETE
        # elif aisle_targeting_state == 3:
        #     # this state is for doing a tiny leftward rotation to adjust
        #     if left_spin_complete is False:
        #         vL = -.4
        #         vR = .4
        #         left_spin_complete = True
        #     else:
        #         vL = 0
        #         VR = 0
        #         aisle_targeting_state = 1
        # if (state == "TURN HEAD LEFT"):
        #     vL=0
        #     vR=0


    if (state == "START"):
        # Move arm back towards self
        ikResults = [0,0,0,0,0.07,0,-1.5,2.29,-1.8,1.1,-1.4,0,0,0]
        # ikResults = [0,0,0,0,0.07,1.02,-1.5,2.29,-1.8,1.1,-1.4,0,0,0]
        moveArmToTarget(ikResults)        
        
        # State Transition
        if checkArmAtPosition(ikResults, 0.00005):
            print("=== AWAITING")
            state = "AWAIT"
        else:
            state = "START"


        # initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
        # ikResults = [0,0,0,0,0.07,0,-1.5,2.29,-1.8,1.1,0,0,0,0]
        # for res in range(len(ikResults)):
        #     if my_chain.links[res].name in part_names:
        #         if abs(initial_position[2]-ikResults[2]) < 0.1 or res == 2:
        #             robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
        #             print("Setting {} to {}".format(my_chain.links[res].name, ikResults[res]))
        # arm_error = 0
        # for item in range(14):
        #     arm_error += (initial_position[item] - ikResults[item])**2
        # arm_error = math.sqrt(arm_error)

        # print("ARM ERROR {}".format(arm_error))
        # if arm_error < 0.00005:
        #     print("TURN HEAD LEFT")
        #     state = "TURN HEAD LEFT"

    if (state == "AWAIT"):
        if(received_shopping_list):
            print("=== TRAVELING")
            state = "TRAVELING"
        
        

    if (state == "TURN HEAD LEFT"):
        robot.getDevice("head_1_joint").setPosition(1.24)

        if robot.getDevice("head_1_joint").getPositionSensor().getValue() > 1.23:
            
            if len(recognized_objects) > 0:
                
                for item in recognized_objects:
                    if target_item in str(item.get_model()):
                        target = recognized_objects[0].get_position()
                        dist = abs(target[2])

                        if dist < 5:

                            print("ITEM FOUND!")
                            state = "ITEM FOUND LEFT"

                            previous_coords = gps.getValues()
                            n = compass.getValues()
                            rad = ((math.atan2(n[0], n[2]))-1.5708)
                            previous_orientation = rad
                        else:
                            print("ITEM FOUND LEFT, TOO FAR AWAY")
                
            if state != "ITEM FOUND LEFT":
                state = "TURN HEAD RIGHT"

    if (state == "TURN HEAD RIGHT"):
        robot.getDevice("head_1_joint").setPosition(-1.24)
        if robot.getDevice("head_1_joint").getPositionSensor().getValue() < -1.23:
            
            if len(recognized_objects) > 0:

                for item in recognized_objects:
                    if target_item in str(item.get_model()):

                        target = recognized_objects[0].get_position()
                        dist = abs(target[2])
                        print(dist)
                        if dist < 5:

                            print("ITEM FOUND!")
                            state = "ITEM FOUND RIGHT"

                            previous_coords = gps.getValues()
                            n = compass.getValues()
                            rad = ((math.atan2(n[0], n[2]))-1.5708)
                            previous_orientation = rad
                        else:
                            print("ITEM FOUND RIGHT, TOO FAR AWAY")
                
                
            if state != "ITEM FOUND RIGHT":
                print("=== FORWARD")
                state = "FORWARD"

    if (state == "FORWARD"):
        vL = 5
        vR = 5
        delay(10)
        state = "TURN HEAD LEFT"

    if (state == "ITEM FOUND RIGHT"):
        target = recognized_objects[0].get_position()
        img_pos = recognized_objects[0].get_position_on_image()
        if img_pos[0] > (CAM_WIDTH/2)+35:
            if(vrb):
                print("MOVING BACKWARD")
            vL = -2
            vR = -2
        elif img_pos[0] < (CAM_WIDTH/2)+25:
            if(vrb):
                print("MOVING FORWARD")

            vL = 5
            vR = 5
        else:
            previous_coords = gps.getValues()
            n = compass.getValues()
            rad = ((math.atan2(n[0], n[2]))-1.5708)
            previous_orientation = rad
            state = "TURN AROUND"

    if (state == "ITEM FOUND LEFT"):
        target = recognized_objects[0].get_position()
        img_pos = recognized_objects[0].get_position_on_image()
        
        if img_pos[0] < (CAM_WIDTH/2)-35:
            if(vrb):
                print("MOVING BACKWARD")
            vL = -2
            vR = -2
        elif img_pos[0] > (CAM_WIDTH/2)-25:
            if(vrb):
                print("MOVING FORWARD")
            vL = 5
            vR = 5
        else:
            previous_coords = gps.getValues()
            n = compass.getValues()
            rad = ((math.atan2(n[0], n[2]))-1.5708)
            previous_orientation = rad
            state = "TURN AROUND"

    if (state == "TURN AROUND"):
        #Rotate robot to center object
        target = recognized_objects[0].get_position()
        dist = abs(target[2])
        img_pos = recognized_objects[0].get_position_on_image()
        robot.getDevice("head_1_joint").setPosition(0)
        robot.getDevice("head_1_joint").setVelocity(0.3)

        headCenter = robot.getDevice("head_1_joint").getPositionSensor().getValue()



        # Set Height of Camera
        if img_pos[1] < 105:
            robotHeightDelta = 0.05
        elif img_pos[1] > 125:
            robotHeightDelta = -0.05
        else:
            robotHeightDelta = 0


        if img_pos[0] > (CAM_WIDTH/2)+2:
            if(vrb):
                print("TURNING RIGHT")
            err = img_pos[0] - CAM_WIDTH/2

            if err > 10:
                vL = 2
                vR = -2
            else:
                vL = 1*(err/10)
                vR = -1*(err/10)
        elif img_pos[0] < (CAM_WIDTH/2)-2:
            if(vrb):
                print("TURNING LEFT")
            err = CAM_WIDTH/2 - img_pos[0]

            if err > 10:
                vL = -2
                vR = 2
            else:
                vL = -1*(err/10)
                vR = 1*(err/10)

        elif dist > 0.65 and headCenter > -0.01 and headCenter < 0.01:
            if(vrb):
                print("MOVING FORWARD")
            vL = 2
            vR = 2
        else:
            if(vrb):
                print("STOP")
            vL = 0
            vR = 0


       
        robotHeight = robot.getDevice("torso_lift_joint").getPositionSensor().getValue()
        

        # print(target[2])
        #Exit
        if dist < 0.65 and abs(headCenter) < 0.01:# and img_pos[1] >= 105 and img_pos[1] <= 125 :

            print("=== IK")
            state = "IK"
        

        
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    # print("SET {}, {}".format(vL, vR))
    robot.getDevice("wheel_left_joint").setVelocity(vL)
    robot.getDevice("wheel_right_joint").setVelocity(vR)

    pass

# Enter here exit cleanup code.
