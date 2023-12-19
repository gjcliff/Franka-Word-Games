# ME495 Embedded Systems Final Project
Team Members: Ananaya Agarwal, Graham Clifford, Ishani Narwankar, Abhishek Sankar, Srikanth Schelbert

## Project Overview
The goal of our project was to use the Franka robot as a facilitator for a game of hangman. Our robot is able to setup a game of hangman (draw the dashes and hangman stand) as well as interact with the player (take word or letter guesses) before writing the player's guesses on the board or adding to the hangman drawing.

In order to accomplish our project's core goals, we developed:
a force control system combined with the use of april tags to regulate the pen's distance from the board, an OCR system to gather and use information from the human player, and a hangman system to mediate the entire game.

## Quickstart Guide - Running the Game on the Robot
1. Install PaddlePaddle using `python -m pip install paddlepaddle-gpu -i https://pypi.tuna.tsinghua.edu.cn/simple`
2. Install paddleocr using `pip install "paddleocr>=2.0.1" # Recommend to use version 2.0.1+`
3. Install Imutils using `pip install imutils`
4. SSH into the student@station computer from the terminal after you are connected to the robot via ethernet cable.
5. On your browser open https://panda0.robot, unlock the robot, and activate FCI
6. Once the robot is unlocked and has a blue light, use `ros2 launch drawing game_time.launch.xml` to open RVIZ and start the hangman game on the robot.

## List of Nodes
1. Brain: 

    Interfaces with all other nodes to evaulate data.
2. ImageModification: 
    
    Modifies images for OCR using opencv.

3. Paddle_Ocr:

    Performs OCR and publishes predictions.

4. Hangman: 

    Plays the hangman game based on the OCR user input.

5. Drawing:

    Plan and execute robot paths using force control.

    Accept requests to plan trajectories for the franka robot, and subsequently send them to another node to be executed. Additionally,calculate the estimated force at the end-effector, and publish it on a topic.

6. TrajectoryExecution: 

    Execute trajectories planned for the franka robot.

    Execute trajectories planned for the franka robot at 10hz. This is a stand in for the MoveIT execute trajectory action, since MoveIT doesn't allow us to cancel goals. When a trajectory is planned by either the MoveGroup motion planner or the compute_cartesian_path service, the result is returned in the form of a RobotTrajectory message. This message includes a JointTrajectory message, which contains a list of JointTrajectoryPoint messages. We must break up this list into individual JointTrajectory messages, with just one element in the JointTrajectoryPoint list. This way, we can execute the origina RobotTrajectory discreetly by publishing JointTrajectories on the /panda_arm_controller/joint_trajectory topic.

7. Kickstart:

    Kickstart handles setting up the board for the hangman game.This includes calibrating the robot to the board, drawing the 5 lines for the word to guess, drawing the 5 lines for the 5 wrong guesses, and drawing the stand for the hangman.

8. Tags:

    This node deals with the april tags and handels tf tree. It publishes a static transform between camera and the robot, takes the arm to a specified pose and looks at the april tags on the board and publishes a board to robot transform, and gives the start pose of any letter with respect to the panda_link0.

## List of Launchfiles
1. game_time.launch.xml:

    This launchfile launches the tags, kickstart, and brain nodes along with the ocr_game.launch.xml and drawing.launch.xml launch files.

2. ocr_game.launch.xml:

    This launchfile launches the paddle_ocr, image_modification, and hangman nodes.

3. drawing.launch.xml:

    This launchfile launches our RVIZ simulation and the april_tag.launch.xml.

4. april_tag.launch.xml:

    This launchfile launches the camera configuration in RVIZ along with the pointcloud information. It also launches the tags node.

## Overall System Architecture

The following diagram illustrates the overall system design and showcases how different nodes interact with eachother in order to accomplish the goals of our project.

![Node_diagram](https://github.com/ME495-EmbeddedSystems/final-project-Schelbert197/assets/42013894/cb9bc600-ac66-48a0-a4ba-3c6fecbadffb)


## Lessons Learned
1. OCR:

    The PaddleOCR model comprises two layers: text detection and recognition. The detection layer searches for text in the image, identifying regions of interest, while the recognition layer identifies the text within these bounded regions, providing predictions with associated confidence values. Our tests indicate that the model struggles to reliably detect single characters in images, seemingly because it is trained on materials such as books and articles that feature large blocks of text. Consequently, it fails to produce accurate guesses in such scenarios. To address this issue, we disable the text detection feature in PaddleOCR and exclusively utilize its text recognition capability.

    We use OpenCV to modify the image before feeding it into PaddleOCR in order to improve accuracy:

    - Convert the image from RGB to grayscale.
    - Apply Gaussian blurring to reduce noise.
    - Employ Canny edge detection to identify edges in the image.
    - Identify closed contours and sort them based on contour area.
    - Iterate over the contours to pinpoint the largest contour approximating a rectangle, which corresponds to the whiteboard.
    - Apply a four-point perspective transform to straighten the image.
    - Binarize the warped image using adaptive thresholding to account for lighting differences.
    - Use the dilation operation to widen the text within the bounded region (only performed for single-character recognition).

    Subsequently, we publish two sets of images that are fed into the OCR model for recognition.

    The OCR model provides predictions with associated confidence values. To refine result accuracy, we've implemented a simple filtering mechanism. This filter assesses predictions from the OCR model, gauging their legitimacy based on both the frequency of a particular prediction and its confidence value. High-confidence guesses pass through quickly, while low-confidence and frequently occurring predictions may experience a slight delay but are eventually processed. However, low confidence and infrequent predictions are prevented from passing through altogether. This approach ensures that our system avoids false-positive predictions, maintaining a more reliable and accurate output.

2. Admittance Control

    Implementing admittance control was challenging to say the least. After we figured out many different wrong ways to achieve admittance control, we settled on a practical but inaccurate method. Our admittance controls works as follows:
    Calculate the expected torque in panda_joint6 due to the weight of the gripper and its attachments.
    Obtain a plane from the april tag attached to the whiteboard. The robot will attempt to draw at points in this plane.
    Subtract this expected torque from the actual torque in panda_joint6, and then convert any difference in these two values into linear force in the end-effector’s frame.
    If we measure a force greater than a certain threshold in the end-effector frame’s x direction, then we must be making contact with the whiteboard. At this point, stop moving the gripper.
    We learned that apparently we cannot cancel action goals in ROS Iron at this time. Because of this, we had to write our own node for executing planned robot trajectories.
    This node works as follows:
    Plan a trajectory, and then repackage that trajectory into a list of JointTrajectory messages, each with one JointTrajectoryPoint.
    Do not forget to change the time_from_start parameter in the JointTrajectoryPoint message to 100000000 nano seconds. If you do not do this, the robot will behave unpredictably.
    Publish each of these JointTrajectory messages one by one on the /panda_arm_controller/joint_trajectory topic in a timer_callback with a 10hz frequency.
    After stopping the gripper, update the plane obtained from the april tag at the beginning of the procedure to be aligned with the point the end effector impacted the whiteboard.
    Plan a trajectory to the next pose in the queue.
    Move to this point with PID admittance control enabled, where the input to the PID loop is the force at the end effector, and the output is the angle of panda_joint6.

    This method is inaccurate, and a little crude. The calculation of the force at the end-effector isn’t always correct, joint torque measurements from the franka itself can be noisy and erratic, and using only the angle of panda_joint6 as the output of the PID loop can cause curved lines drawn on the whiteboard. In the future, a better implementation would be to write a c++ node that completely controls the franka, alongside a ros_control c++ file that implements the admittance control. I’m not sure if both or only one of these would be necessary. Libfranka does a lot more math for you than MoveIT does which is essential for admittance control. 

    In addition, our group was running the moveit.launch.py file for the franka on our own pcs, which was a huge mistake because we were missing out on the benefits of having a real time operating system controlling the robot. I (Graham) take full responsibility for this.

## Future Work
Although our team redesigned a spring force control adapter for the Franka gripper for our fallback goal, we quickly outgrew the need for it as we began trying to implement force control. However, our project could still be significantly improved on the force control side of things. By the demo day, our team had achieved using force control to draw characters on the board but could have improved the quality of writing with more time to tune the force control parameters.

Along with improving force control, our team envisioned incorporating an extra part of the game where the robot picks up different colored pens depending on whether the player's guess is right or wrong. Due to the amount of people on our team, we actually designed and manufactured the pen stand and began writing the code to implement this part of our project. We had finalized our gripper code and had begun calibrating the robot to our april tag on the pen stand. We unfortunately ran out of time incorporating this into our final gameplay.  

Our final stretch goal that we would love to improve our project with is changing the board position between the player's turns. Due to our use of april tags and force control this could have been implemented with additional time to properly calibrate the camera's exact position with respect to the board and robot.

## Demonstration Videos
The following video showcases a full runthrough of our project. In it, we demonstrate the robot's ability to calibrate and determine the position of the board, run through the game setup sequence while using force control, and interact with the player and receive both single letter and full word guesses.

[![Full Project Demo](https://img.youtube.com/vi/-JOk2Mi-f9s/0.jpg)](https://www.youtube.com/watch?v=-JOk2Mi-f9s "Full Project Demo")