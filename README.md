# srcsim_docker
Docker images for srcsim

### Instructions
1. Install [docker](https://docs.docker.com/cs-engine/1.12/#install-on-ubuntu-1404-lts-or-1604-lts)
2. create a network bridge named srcsim
	```bash
	docker network create --subnet 192.168.1.0/16 --driver bridge srcsim
	```
3. Install nvidia-docker plugin `https://github.com/NVIDIA/nvidia-docker/wiki/Installation` 
4. build the docker image 
	```bash
	git clone https://github.com/WPI-Humanoid-Robotics-Lab/srcsim_docker.git
	cd srcsim_docker
	docker build -t srcsim0.9 .
	```
5. run the docker image
	```bash
	bash run_srcsim_docker0.9.bash
	```
6. connect gazebo client on host machine (in a new terminal)
	```bash
	GAZEBO_MASTER_URI=http://192.168.2.10:11345 gzclient
	```
7. To run code on docker image add the following 2 variables to ~/.bashrc
	```bash
	export ROS_MASTER_URI=http://192.168.2.10:11311
        export ROS_IP=192.168.0.1 # Confirm this from ifconfig results
        rostopic list
	```


