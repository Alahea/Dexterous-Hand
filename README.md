# Overview
The Dexterous-Hand ROS2 project integrates EMG sensor and the ROKOKO Smartglove to enable robotic grasping control within a ROS2 environment. The phases are: record a task with the Rokoko Smartglove, do this by recording data for each pahse of the task or record the entire dataset and then split into the number of phases. Save Rokoko Smartglove data as a CSV file, streaming the data is also possible using the IP address. Then train EMG data following the order: udp data collector, feature extractor, train, live classify. Then run the inspire hand listener, once connected you can communicate with the Inspire Robot. Use examples in the inspire_hand files as samples to test. 

# Getting Started
Before running the system, ensure the following components are set up:

## EMG Setup
Follow the installation steps for libEMG: libemg-ros on GitHub
Note: A different training method will be used in this project, but completing the libEMG installation is recommended. Some packaged might need to be modified depending the system you are using.
Here is how to train the gForcePro Oymotion Armband:
1. EMG_udp_collector.py
    Note: for best results, collect within the range [15,000, 25,000] the higher the data is for each guesture the better the training will be. When recording each guesture, make sure to have the EMG Armband as high on the forearm as possible. Keeping each muscle steady when recording is also recommended.
3. EMG_feature_extractor.py
4. EMG_train_model.py
   We are using the random forest model to train data.
6. EMG_live_classifier.py
   This will both visualize the tranined guestures while also publishing it to ROS.

## ROKOKO Smartglove Setup
Recommended to use Mac or Windows for faster processing.
A 5G internet connection can significantly improve the setup and data transmission speed.

# Controlling the Inspire Hand
communication with the inspire hand is done through inspire_hand_listener.py
