# Perception

## Advantech setup

1. c++ version update (ver. 9.4)
    ```bash
    sudo apt install build-essential
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt update
    sudo apt install gcc-9 g++-9
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-9
    gcc --version
    ```
> dpkg error

    ```bash
    sudo su
    rm /var/lib/dpkg/info/format
    echo "1.0" > /var/lib/dpkg/info/format
    dpkg --configure -a
    apt-get update
    apt-get upgrade
    ```
    
2. cmake version update (ver. 3.16.3)
    ```bash
    wget https://cmake.org/files/v3.16/cmake-3.16.3.tar.gz
    tar -xvzf cmake-3.16.3.tar.gz
    cd cmake-3.16.3
    ./bootstrap --prefix=/usr/local
    make
    make install
    cmake --version
    ```
 

## ROS install

>These instructions will install **ROS melodic**, which is available for Ubuntu Bionic (18.04)

### ROS noetic install
1. 설치 패키지 목록에 ROS를 추가합니다.
	```bash
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	```

2.  ros 설치 키를 설정합니다.
	```bash
	sudo apt install curl
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	```

3.  패키지 목록을 업데이트 합니다.
	```bash
	sudo apt update
	```

4. 데스크탑 풀버전을 설치합니다.
	```bash
	sudo apt install ros-melodic-desktop-full
	```

5.  Shell에서 실행할 수 있도록 환경설정을 합니다.
	>_[bash]_
	```bash
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	```

6.  ros를 실행합니다.
	```bash
	roscore
	```

### ROS noetic setup
1. Workspace 생성
	```bash
	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/
	catkin_make
	```
	
2. System 환경 설정
	```bash
	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
	echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
	echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
	echo "alias cs='cd ~/catkin_ws/src'" >> ~/.bashrc
	echo "alias cw='cd ~/catkin_ws'" >> ~/.bashrc
	echo "alias cm='cd ~/catkin_ws && catkin_make'" >> ~/.bashrc
	
	source ~/.bashrc
	```

### Install dependencies
1. pcl
	```bash
	sudo apt-get install libpcl-dev
	```
2. boost
	```bash
	sudo apt-get install libboost-all-dev
	```
3. eigen
	```bash
	sudo apt-get install libeigen3-dev
	```
4. jsoncpp
	```bash
	sudo apt-get install libjsoncpp-dev
	```
5. pybind11
	```bash
	pip install pybind11
	pip install "pybind11[global]"
	```


## S/W Download link

### Git install
1. Install Git
	```bash
	sudo apt install git
	```

### Download link
>Git clone repository
```bash
cd ~/catkin_ws/src
git clone {Download_link}
```
- Webviewer_download_link : [Webviewer](https://github.com/soslab-solution/webviewer-ros.git)
- Perception_ROS_download_link : [ROS](https://github.com/soslab-solution/Perception_ROS.git)
- Perception_API_download_link : [API](https://github.com/soslab-solution/Perception_API.git)
- MLX_download_link : [MLX](https://github.com/soslab-solution/mlx_ros_driver.git)

### Configuration 
```
catkin_ws/
  |--build/
  |
  |--devel/
  |
  |--src/
     |--mlx_ros_driver/
     |--Perception_ROS/
     |--Perception_API/
     |--webviewer-ros/
 ```


## Webviewer install

### Client install

1.  Install nodejs
	```bash
	curl -sL https://deb.nodesource.com/setup_16.x -o nodesource_setup.sh
	sudo bash nodesource_setup.sh
	sudo apt-get install nodejs
	```

2.  Install react app
	> Install npm packages
	```bash
	cd ~/catkin_ws/src/webviewer-ros/
	npm install
	npm run build
	sudo npm install -g serve
	```

### Server install

1. install rosbridge package
    ```bash
    sudo apt-get install ros-melodic-rosbridge-suite
    cm
    ```

### Perception_API build

1. build Perception_API
    ```bash
    cd ~/catkin_ws/src/Perception_API/
    sh ./build.sh
    ```

2. so file copy to Perception_ROS
    ```bash
    cp -f ~/catkin_ws/src/Perception_API/bin/include/* ~/catkin_ws/src/Perception_ROS/include/perception_lib/
    ```

## Perception Run

### ROS Run
1. run ROS Master
    ```bash
    roscore
    ```

### ML Run
1. run ML-X Driver
    ```bash
    roslaunch ml ml.launch
    ```

### Server Run
1. run Server
    ```bash
    roslaunch obj_detect_3d_ros web_obj_detect_3d_ros.launch
    ```

### Client Run

1. run App (viewer)
    ```bash
    cd ~/catkin_ws/src/webviewer-ros/
    serve -s build
    ```
    - chrome localhost:3000 접속

