# Perception

## ROS install

>These instructions will install **ROS Noetic**, which is available for Ubuntu Focal (20.04)

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
	sudo apt install ros-noetic-desktop-full
	```

5.  Shell에서 실행할 수 있도록 환경설정을 합니다.
	>_[bash]_
	```bash
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	```

6.  ros를 실행합니다.
	```bash
	roscore
	```

### ROS noetic setup
1. Workspace 생성
	```bash
	mkdir -p ~/ros_ws/src
	cd ~/ros_ws/
	catkin_make
	```
	
2. System 환경 설정
	```bash
	echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
	echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
	echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
	echo "alias cs='cd ~/ros_ws/src'" >> ~/.bashrc
	echo "alias cw='cd ~/ros_ws'" >> ~/.bashrc
	echo "alias cm='cd ~/ros_ws && catkin_make'" >> ~/.bashrc
	
	source ~/.bashrc
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
cd ~/ros_ws/src
git clone {Download_link}
```
- Webviewer_download_link : [Webviewer](https://github.com/soslab-solution/webviewer-ros)
- Server_download_link : [Server](https://github.com/soslab-solution/obj_detect_3d_ros)
- MLX_download_link : [ML](https://github.com/soslab-solution/mlx_ros_driver)

### Configuration 
```
ros_ws/
  |--build/
  |
  |--devel/
  |
  |--src/
     |--ml/
     |--obj_detect_3d_ros/
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
	cd ~/ros_ws/src/webviewer-ros/
	npm install
	npm run build
	sudo npm install -g serve
	```

### Server install

1. install rosbridge package
    ```bash
    sudo apt-get install ros-noetic-rosbridge-suite
    cm
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
    cd ~/ros_ws/src/webviewer-ros/
    serve -s build
    ```
    - chrome localhost:3000 접속


## Advantech setup

1. c++ version update (ver. 9.4)

