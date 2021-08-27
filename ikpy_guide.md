So, IKPY is surprisingly not as robust a library as I would have imagined, and its quite poorly documented. After struggling for a few days trying to get it to work with TIAGO Steel, here are some things that helped.


*   So, IKPY is actually a bit outdated. Optionally, I'd recommend installing a fork of IKPY by user Alters-Mit which fixes some issues with prismatic joints, which the TIAGO Steel contains. More info here. https://github.com/Phylliade/ikpy/issues/96
    *   To install this fork of ikpy, run `pip install git+https://github.com/alters-mit/ikpy.git#egg=ikpy` in your console.
    *   I'm not sure if this helped, but it seemed to. If you don't have problems, do this optionally.
    *   Finally, put the following at the top of your controller file-
    *   ```Python
        from ikpy.chain import Chain
        from ikpy.link import OriginLink, URDFLink
        ```
*   Once we have IKPY installed, get the URDF file for your robot. This can be done using the example code in the \`inverse\_kinematics.wbt\` world, but changing instances of supervisor to instances of robot.
    
    *   ```Python
        #Alternatively, here is my code for creating the URDF
        
        with open("tiago_urdf.urdf", "w") as file:  
            file.write(robot.getUrdf())
        ```
        
*   I’d recommend doing this once, then rewriting/commenting out the sample code so the file does not change. Webots attaches weird ID numbers to parts which change every time the world is reloaded. By keeping a static URDF file, you avoid this.
*   Next, we have to create the chain for the robot. A common problem with IKPY is that it doesn't consider end effectors. Well did you know there was a poorly documented option for actually considering them? It's called the `last_link_vector` option. Let's see how to use it for proper results.
*   First, let's create the chain normally. We need a list of base nodes, which are basically joints in the URDF with no parents. This can be done manually, but thanks to another poster I have a list of these for the TIAGO Steel.
    *   ```Python
        # Replace the '#####' with the unique 5 digit id found in your URDF
        base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_#####_joint", "TIAGo front arm_#####"]
        ```
*   Once we have the base nodes, the code to create a chain is trivial. Simply do the following synta, with your own list of `base_elements`.
    *   ```Python
        my_chain = Chain.from_urdf_file("our_urdf.urdf", base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_11367_joint", "TIAGo front arm_11367"])

        print(my_chain.links)
        ```
*   Perfect! Now when you do `print(my_chain.links)`, you should see the output in your Webots console.
    *   ```Python
        [Link name=Base link bounds=(None, None), Link name=base_link_Torso_joint bounds=(None, None), Link name=torso_lift_joint bounds=(0.0, 0.35), Link name=torso_lift_link_TIAGo front arm_11367_joint bounds=(None, None), Link name=arm_1_joint bounds=(0.07, 2.68), Link name=arm_2_joint bounds=(-1.5, 1.02), Link name=arm_3_joint bounds=(-3.46, 1.5), Link name=arm_4_joint bounds=(-0.32, 2.29), Link name=arm_5_joint bounds=(-2.07, 2.07), Link name=arm_6_joint bounds=(-1.39, 1.39), Link name=arm_7_joint bounds=(-2.07, 2.07), Link name=arm_7_link_wrist_ft_tool_link_joint bounds=(None, None), Link name=wrist_ft_tool_link_front_joint bounds=(None, None), Link name=gripper_right_finger_joint bounds=(0.0, 0.045)]
        ```
*   Now, let's take the very last element in that list, in this case, `gripper_right_finger_joint` and find it in our URDF file, which can be opened in a text editor of your choice.
    *   ```XML
        <joint name="gripper_right_finger_joint" type="prismatic">
            <parent link="front"/>
            <child link="gripper_right_finger_link"/>
            <axis xyz="1 0 0"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <limit effort="16" lower="0" upper="0.045" velocity="0.05"/>
        </joint>
        ```
    *   As we can see, this joint still has a `child`, and this `child` is actually our end-effector! (In this case, the right "hand" of the TIAGO Steel)
    *   Since this child is not included in the IKPY chain, it is being excluded from calculations. But using `last_link_vector`, we can change this.
    *   First, lets get the dimensions of the hand by finding `gripper_right_finger_link` in the URDF.
    *   ```XML
        <link name="gripper_right_finger_link">
            <visual>
            <origin xyz="0.004 0 -0.1741" rpy="0 0 0"/>
            <geometry>
                <box size="0.0076 0.05091 0.08982"/>
            </geometry>
            </visual>
            ...
        </link>
        ```
    *   The actual object is much larger, but all we are interested in is either the `visual` or `collision` properties. In this case, they are the same.
    *   From here, we should take note of the origin xyz vector `[0.004, 0, -0.1741]` for the hand, this will be our `last_link_vector`
*   Finally, let's add the `last_link_vector` option to the creation of our chain.
    *   ```Python
        my_chain = Chain.from_urdf_file("our_urdf.urdf", last_link_vector=[0.004, 0,-0.1741], base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_11367_joint", "TIAGo front arm_11367"])
        print(my_chain.links)
        ```
    *   And you should now see the output included one last link, representing the end effector.
    *   ```Python
        [Link name=Base link bounds=(None, None), ..., Link name=gripper_right_finger_joint bounds=(0.0, 0.045), Link name=last_joint bounds=(None, None)]
        ```
*   Okay! Now that we have the chain created, we need to deactivate any links that we cannot control. While this can be done manually, here is the code that I use to do this.
    *   ```Python
        # The Tiago robot has multiple motors, each identified by their names below.
        # Make sure to use a parts list specific to your robot's controllable joints
        part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
                    "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
                    "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

        for link_id in range(len(my_chain.links)):

            # This is the actual link object
            link = my_chain.links[link_id]
            
            # I've disabled "torso_lift_joint" manually as it can cause
            # the TIAGO to become unstable.
            if link.name not in part_names or  link.name =="torso_lift_joint":
                print("Disabling {}".format(link.name))
                my_chain.active_links_mask[link_id] = False
*   Of course, we also need to initialize our motors. I've adapted sample code from `inverse_kinematics.wbt` in order to do this myself.
    *   ```Python
        # Initialize the arm motors and encoders.
        motors = []
        for link in my_chain.links:
            if link.name in part_names and link.name != "torso_lift_joint":
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
        ```
*   Now we are FINALLY ready to do inverse kinematics. The first thing we need to do is get the current position of our links. For all disabled links, this is `0`. We can get the active ones by using the position sensors we just enabled.
    *   Following more sample code from `inverse_kinematics.wbt`, here was my solution for the TIAGO chain,
        which has 4 disabled links at the front and at the end. You can use `print(my_chain.links)` to see the order of links, which is very important.
    *   ```Python
        initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
        ```
*   Next, we need our target for the arm in robot coordinates. This particular chain for the Tiago sets 0,0,0 at the base of the robot, with X going forward, Y going left, and Z going up. This may be different for your robot.
    *   In my case, I used a Camera with recognition to get a target. I could write up a whole thing on how I managed to get this work, but for now, here is the code I used to get my target. Obviously, yours will be different depending on what your robot does
    *   ```Python
        recognized_objects = camera.getRecognitionObjects() # Refer to the recognition section is Webots doc for more info on this
        target = recognized_objects[0].get_position() # This is the position of the target in camera coordinates
        offset_target = [-(target[2])+0.22, -target[0]+0.08, (target[1])+0.97+0.2] # And here it is translated to robot/IK coordinates
        # Note that the `+0.2` is optional, and since Z is up, this raises the target IK by 20cm for my solution.
        ```
*   Finally, we run the IK function to get a result. You can optionally orient it, which is best done through trial and error as I haven't totally figured it out myself, haha.
    *   ```Python
        ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position,  target_orientation = [0,0,1], orientation_mode="Y")
        ```
    *   This orientation keeps the end effector parallel to the ground, so I can use the claws to pick things up from the side.
    *   Note I don't use `max_iter` since it decreases the accuracy of the IK.
    *   Make sure NOT to call this function on every timestep (this will create massive lag), but only when the accuracy is off. I use a squared distance with a tolerance of `0.005`.
        *   ```Python
            # When error > 0.005, I rerun ik calculations.
            # You can either use forward kinematics or just the previous target for `ikTarget`
            # I use the latter as its faster, even if its technically less accurate
            error = 0
            for item in range(3):
                error += (offset_target[item] - ikTarget[item])**2 #ikTarget is the previous target used for calculations.
            error = math.sqrt(error)
            ```
*   And once you have the results, you apply them to your motors.
    *   Again, this can be done manually, or automatically.
    *   ```Python
        for res in range(len(ikResults)):
            # This if check will ignore anything that isn't controllable
            if my_chain.links[res].name in part_names:
                robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
                print("Setting {} to {}".format(my_chain.links[res].name, ikResults[res]))
        ```
*   If all goes well that should be it! But here are some tips for best results.
    *   Remember that IK calculations are done in robot coordinates, relative to the robot's base, or more accurately, the Base_link from your URDF/Chain.
    *   You can visualize your IK result with the following code
        *   ```Python
            import matplotlib.pyplot
            from mpl_toolkits.mplot3d import Axes3D
            ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

            # Plot the current position of your arm
            my_chain.plot(initial_position, ax, target=ikTarget) #Also, ikTarget=offset_target, sorry for any confusion

            # And plot the target position of your arm
            my_chain.plot(ikResults, ax, target=ikTarget)
            matplotlib.pyplot.show()
            ```
        *   This is extremely helpful when debugging, because it can show whether IK is failing or your
            target is off. Remember- IK is a heuristic solution- so it won't be perfect always.
    *   In the case of the TIAGO, the `last_link_vector` adds the right "finger" into consideration, so it will place the right finger on your target. If you want to actually grab something, you'll have to move your target slightly to the right of your object.
    *   Orientation isn't exactly perfect, so don't worry if it seems like its not working perfectly...because it's not. This is expected behaviour from IKPY. Some people online have said that running IK calculations once for position (without orientation) and then again with orientation can solve this...but I haven't tried this myself.
    *   Let me know if you need more help getting your Camera/IK working together, this took me a while and getting all the coordinate system changes and translations was rough, since Webots likes to lie about the actual position of things sometimes...
    *   Check out these helpful posts as well
        *   https://piazza.com/class/kjq360pc7ic2xn?cid=147
        *   https://piazza.com/class/kjq360pc7ic2xn?cid=144

Thanks for reading! I know this is late...but I hope it helps someone because honestly, this was not fun :D