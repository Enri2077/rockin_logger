=========================================
            rockin_logger
Luca Iocchi 2015 (iocchi@dis.uniroma1.it)
=========================================


This package contains code and launch files that can be used as examples and templates to implement the logging features required for the RoCKIn benchmarking.

You must adapt this code for your robot.


  
Configuring, logging and testing audio acquisition
==================================================

1. Installation

First, you probably have to install some packages

  sudo apt-get install ros-indigo-audio-common
  sudo apt-get install gstreamer0.10-tools
  sudo apt-get install gstreamer0.10-alsa

2. Check audio settings

Check that the audio settings in the PC where the microphone is connected are properly set.
An easy way is to use 'audacity' program to record and play something.



Testing on stage
================

In order to test the logging/playing function on Stage simulator, download the 'stage_environments' ROS package, compile it in a catkin workspace and do the following steps:

1. Run the simulation thorugh the start_simulation.py script in stage_environments/scripts.
Select peccioli@Home/@Work, sapienzbot, amcl, move_base, RViz and click the Start button.

2. Record a bag 

  $ cd to rockin_logger/bags 
  $ ./recbag.sh <teamname> <benchmark>
  
  teamname = your team name
  benchmark = {TBM|FBM}{H|W}{1..3} (e.g., TBMH1)

3. Drive the robot with 2D Nav Goals in RViz and speak to the microphone.

4. Stop recording the bag (CTRL-C in the terminal where you run the recbag script) and quit the simulation (Quit button in the start_simulation GUI).

5. Re-play the bag 

  $ cd to rockin_logger/bags 
  $ ./playbag.sh <bagfile> [<mapname>]
  
  mapname = {peccioli@Home, peccioli@Work}, default: peccioli@Home.

You will see the logged data in RViz (tf, laser scan, images, point clouds, ...) and listen to the recorded audio from the speakers of your PC.





