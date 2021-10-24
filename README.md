<div id="top"></div>

## About The Project

This is an sample package of Ros for IMU( MPU9250 ) depend on [RTIMULIB](https://github.com/jetsonhacks/RTIMULib)

<p align="right">(<a href="#top">back to top</a>)</p>

### Built With

+ Qt
+ Ros
+ [RTIMULIB](https://github.com/jetsonhacks/RTIMULib)

<p align="right">(<a href="#top">back to top</a>)</p>

## Getting Started

You have to install Qt, Ros and RTIMULIB to use this package.

### Prerequisites

+ Qt
  ```sh
  sudo apt-get install libqt5sql5-mysql
  sudo apt install build-essential
  sudo apt install qtcreator
  sudo apt install qt5-default
  ```
  
+ RTIMULib
  ```sh
  cd ~
  git clone https://github.com/jetsonhacks/RTIMULib.git
  ```
  
  To use this package, it is necessary to install RTIMULib using the CMake method:
  ```sh
  cd RTIMULib/Linux
  mkdir build
  cd build
  cmake .. -DBUILD_DEMO=0 -DBUILD_GL=0
  make -j4
  sudo make install
  sudo ldconfig
  ```


### Installation

get to your workspace/src then clone and build the package:
```sh
cd ~/catkin_ws/src
git clone https://github.com/Mes0903/mes_imu.git
cd ..
catkin_make
source devel/setup.sh
```

<p align="right">(<a href="#top">back to top</a>)</p>

## Usage

use roslaunch to run this package:
```sh
roslaunch mes_imu mes_imu.launch
```

you can use `rostopic echo` to check if it's working:
```sh
rostopic echo /imu
```

also you can run the subscriber node to check if it's working:
```sh
rosrun mes_imu mes_imu_lis
```

<p align="right">(<a href="#top">back to top</a>)</p>

## Acknowledgments

+ [roscpprtimulib](https://github.com/Garfield753/roscpprtimulib?fbclid=IwAR0VPKFQcrB7JmQiaBDjzXnwbvWkOMG_DUAKlCwOwMHAH6AX9eJfj6Mx_qs)
+ [RTIMULIB](https://github.com/jetsonhacks/RTIMULib)

<p align="right">(<a href="#top">back to top</a>)</p>
