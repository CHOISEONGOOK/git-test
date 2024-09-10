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
- Perception_Viewer_download_link : [Perception_Viewer](https://github.com/SOSLAB-github/Perception_Viewer.git)
- Perception_Engine_Safety_download_link : [Perception_Engine](https://github.com/SOSLAB-github/Perception_Engine_Safety.git)
- Perception_API_Safety_download_link : [Perception_API](https://github.com/SOSLAB-github/Perception_API_Safety.git)
- Mlx_Ros_Driver_download_link : [Mlx_Ros](https://github.com/SOSLAB-github/Mlx_Ros_Driver.git)


### ML-X SDK Download link
>Git clone repository
```bash
cd ~/Documents
git clone {Download_link}
```
- Mlx_SDK_download_link : [Mlx_SDK](https://github.com/SOSLAB-github/ML-X_SDK.git)

### Configuration 
```
ros_ws/
  |--build/
  |
  |--devel/
  |
  |--src/
     |--Mlx_Ros_Driver/
     |--Perception_Engine_Safety/
     |--Perception_API_Safety/
     |--Perception_Viewer/
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
	cd ~/ros_ws/src/Perception_Viewer/
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

## ML / Perception API Build

### ML-X SDK build

### Perception_API build

1. build Perception_API
    ```bash
    cd ~/ros_ws/src/Perception_API_Safety/
    sh ./build.sh
    ```

2. so file copy to Perception_ROS
    ```bash
    cp -f ~/ros_ws/src/Perception_API_Safety/bin/include/* ~/ros_ws/src/Perception_Engine_Safety/include/perception_lib/
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
    roslaunch perception_ros web_perception.launch
    ```

### Client Run

1. run App (viewer)
    ```bash
    cd ~/ros_ws/src/Perception_Viewer/
    serve -s build
    ```
    - chrome localhost:3000 접속

## Remove Source Code

### Remove Perception_API

1. Remove Perception_API
    ```bash
    rm -rf ~/ros_ws/src/Perception_API_Safety
    ```
    
### Remove Webviewer src

1. Remove Webviewer src
    ```bash
    rm -rf ~/ros_ws/src/Perception_Viewer/src
    ```
