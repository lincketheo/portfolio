# Theo Lincke Software Engineering and Applied Math Portfolio

## See the website here: tlincke125.github.io

This is a vast summary of my experience as an applied math and computer science major at CU Boulder. The intention is not to show every project, but to include a project from various fields from pure math to  fun personal projects. Each project is labeled with a skill set applied to it.

You can interact with every one of these projects on Linux (They all work on Arch and Ubuntu) with very minimal setup (except for an OpenCV build for some). I understand that many professionals have limited time, so if there is any required setup, it is just a few lines of bash code that you can run. 

Windows Support for most of these projects is non existent.

## 1. (Pure Math) 
### Riemann Hilbert Problems and Inverse Scattering (*./RH_Inverse_Scattering*)

The folder RH_Inverse_Scattering highlights a pure mathematics project I did as a Sophomore. The highlight of this project is the paper I wrote with two other classmates on complex methods for solving a few nonlinear wave equations. The paper has enough information to understand the relevance of the project. 

## 2. (Applied Math) 
### SVD Image Compression / Recognition (*./SVD_Image_Compression*)
The folder SVD_Image_Compression highlights a project I did as a Freshman using Matlab. The Highlight of this project is the paper I wrote with two other classmates on image compression and facial reconstruction. The paper has enough information to understand the relevance of the project.

## 3. (Fun Personal Project) 
### Any Dimensional Rubiks Cube Solver (*./rubiksCube*)
This is a project I worked on for fun my Freshman year. Coding standards / clean principles followed are fairly minimal (I had no idea how to make a clean application at the time). All it does is solve a logical rubiks cube - I implemented some javascript at one point and a 3d javascript library, but it wasn't that great. Simply type 
```bash
cd ./rubiksCube
# sudo apt install default-jre # Install java
./run 20 1000 # Solve a 20x20 rubiks cube after 1000 randomized scramble moves
```

## 4. (Large Complicated Project and Networking) 
### Sentinet (*./sentinet*)

This is a cross language socket based communication network for robotics inspired by ROS.
This is a very large project so it requires a bit more set up, but the following instructions will guide you through a very simple three part publisher / proxy / subscriber. You will run the publisher in python and the subscriber in C++ (to showcase the multi language and serialization tools). 
Assuming OpenCV is already build

#### Setup / Dependencies
``` bash
# Dependencies
$ cd ./sentinet/sentinet_cpp
$ sudo apt install cmake curl libcurl4-gnutls-dev autoconf automake libtool g++ unzip libzmq3-dev check libgtest-dev
# Install april tags
$ cd ./sentinet_cpp/third_party
$ tar xvf apriltag-3.1.1.tar.gz
$ cd apriltag-3.1.1
$ cmake . && sudo make install
# Build the C++ code
$ cd ../../ #sentinet_cpp root
$ make
# Install python dependencies
$ pip3 install zmq
```
#### Running The Code
##### C++ (subscriber and proxy)
Open 2 terminals (1 for the subscriber and 1 for the proxy)
Run the subscriber
```bash
$ cd <path_to_sentinet_cpp>/build/x86_64/core/bin
$ ./kermit_kernel --debug on # Debug on tells it that there is no serial communication to be made yet with a microcontroller
```
Run the Proxy in a separate terminal
```bash
$ cd <path_to_sentinet_cpp>/build/x86_64/core/bin
$ ./kermit_proxy --cmd_vel --time -1 # -1 = infinite time, press ctrl + C to quit or change time to a time in seconds
```
##### Python (publisher)
In another (third) terminal, execute:
```bash
$ cd <path_to_sentinet_py>
$ python3 main.py
```
Note that this runs for 10 seconds

#### What am I seeing
Assuming everything works, you will see proxy print out a hexadecimal value (the message) and the subscriber (kermit_kernel) should print the same hex value and two numbers. The two numbers are the interpretation of the message as a linear and angular value. Note that the messages are sophisticated enough that end hosts don't need to assume the structure (they have headers and a logical format to represent various data types - see sentient_msg_pkg for the message library). 

The project is using my own custom serialization protocol in c, which consolidates messages into small bite sized chunks.

## 5. (Machine Learning) 
### Neural Network From the ground up in C++ (*./cTensor*)
This was a project I did my freshman year on constructing a neural network using C++. I did not use any native linear algebra libraries (opencv is just for reading the images), and I wrote RREF algorithms, multiplication / addition etc. from scratch in src/Kernel.cpp.

The setup for this project is a lot more lengthy due to the required mnist image database. However, the linear algebra library I wrote works just fine.

```bash
cd ./cTensor
mkdir build
cd build
cmake ../src
make
./run # Has instructions - use option 1, option 2 is a back propigation example that requries thousands of image datasets
```

## 6. (Numerical Analysis)
### Nonlinear ODE Solver using RK4 (*./nonlinear_ODE_solver*)
This is a python module using RK4 to solve nonlinear (or linear) system of differential equations. I have included a few papers in the folder called ./examples on some interesting systems for the fascinated reader. Otherwise, the example module is ready to be executed
```
cd nonlinear_ODE_solver
sudo apt install python3-pip
pip3 install matplotlib numpy
python3 example.py
```
