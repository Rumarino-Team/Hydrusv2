## Submarine Hustle:


This is the description of the latest Rumarino code. This repository is intended to be all-in-one code repo where eveything is connected.

### Disclaimer:

One of the problems we tried to work on was that certain systems were extremely complicated, and we needed help to fully integrate some of them into the Submarine application. After failing to implement the uuv_similation framework into our submarine logic, we understood that we needed a new way to do things. This is the purpose of this repository, We have 3-4 weeks to finish this work by counting the days. This is also an explanation of the things we need to be working on. Another disclaimer is that at first, I thought I could finish this work by myself, but I had some of a breakdown and realized that this would take me more time. 


Explanation of the code and Objective of this project. There are 5 systems we need to complete and test in order to this to work. There are the following. Detector, Controller, Embedded, Mission and Deployment. Right now, there are different tasks that need to be completed by everyone.

A quick explanation of everything will be made in the issues of the project.


### Testing the project:
    
 One of the problems we had last semester was the difficulty of improving and running the codes. I am one of the culprits for that; I wanted to imitate some of the systems we were using and added complexity in certain places that we didn't need. One of the core objectives I want for this project is to improve the dockers and the development process. Now, for developing, you will not need to use the uuv_simulator, tensorRT or make C++ integrations on nothing, and instead, we want to run the project without GPU or extreme computation resources ( this is done mainly by not using the simulator. developing everything in Python).

#### Running the code:

There are two main operating systems: Windows and Linux. Either way, the core thing that we need to use to make this usable is Docker. Please follow the installation instructions for your operating systems.

After that, we want to activate the submarine with this command:

```bash
docker-compose up
```

This means that everything must be configurable with a global `docker-compose.yml` file. Before contributing, ensure that everything you put inside the repo does not crash when running the docker-compose.yml. I will say more later on in this section.



I need to create some type of roles for contributing to the project based on our necessities:

- Detection_Node package developer: We want to integrate easy Python repositories and create more information detections. 

- Embedded_Node package: we want to test that the connection between embedded and controls works correctly. Ensure it obtains the correct data and executes the values correctly.

Mission_Node: Given the sensor information we are obtaining, we want to be able to create mission logic plans.

- Deployment: Someone who knows very well how the embedded systems work and the operating systems of the Jetson. Is able to debug Cuda problems with the Vision node and how to use Linux. Is able to use Docker and understand it very well how it works.





### Contributions:

    
 I realized that I would need the much help as possible from everyone in the team. From now on I will be daily contributing to this repository. Now I have a little more of time because of the 4rth of july and finish some of the heavy duty things on other projects and on my intership. Please this new repository is build with a lot of documentation and issues explaining whast we need to do. I will try to put issues and make more project managment. But the help from eveyone is appreciated.



 #### Dockers and devices and tools:

 In this part I will try to explain my workflow for developing. Please at least you have already a working developing process I will recommend you to use my exact instructions. In this way, you can mitigate a lot of errors that could slow you down.

 - 1 First step is to download Docker. I will link you this and export you to follow all the steps. remember that if you are developing with Windows you have to install first WSL. Sometimes it comes by default but if not it can be a bit tedious to get it together. Either way I recoomend you to install the ubuntu application from the windows installer shop and follow the process.

 Link : https://docs.docker.com/engine/install/

 - Second step: Download Visual Studio code. If youre using any other IDE its either because youre already have experience with that IDE. Either way its mainly on you but I will extremely recommend you to use VSCODE just because I use it and you want to be able to access Docker or other Python tools  I can help you with that.


- Third Step is to downlaod the Docker and Python Extensions and Cody extension in your vscode. 


###  Running without Connected Devices:
 Right now the only functionality that interest me are the Movement , and Vision Devices. This means basically the Camera and the Thrusters. Eveything else is just not going to be implemented. We have a DVL, IMU, Sonar systems and more. But neither of those I have test before  and we do not have the time frame to work on it efficiently.


 How we can test The Camera nd Thruster Dependent Devices if do not  have them?

 Thats how Ros is going to help us. Specially using a tool called rosbag. First we have to understand  the type of devices we have. We have the camera that is a StereoLab Zed2i camera. Cameras are Input devices this means that we get information from it. In the other hand  we have the trhsuters that are output devices. This means that we send them information to execute an action. Given  that robags are recordings of ros publishers information. ROS is the system behind the robots  archuitecture communication. The idea of is that each part of the project have their own ROS pakckage and is able to create ROS nodes that are applications that can either read or  publish information.


 This means that for testing the code we only need the zed2i camera rosbag. I already I have one avialble if anyone want to test it. I have a link to download it over this google drive link: 

 https://drive.google.com/file/d/16Lr-CbW1rW6rKh8_mWClTQMIjm2u0y8X/view?usp=drive_link

 With this file, you will be able to interact with Ros independently if the camera is connected or if you have their dependencies.
 An example of running the rose bag is the following. Make sure to have ros activated first. with the command `source /home/catkin_ws/devel/setup.bash`

 ```bash
    roslaunch detector_node zed_rosbag.launch
 ```


 I will say that the main developers in this project are 
 @Cruiz102 and @JuandelPueblo.  Please get in touch with them if you have any type of doubt. At least for me, I'm available at any moment, at any hour. Please get in touch with me if you're able to contribute.


