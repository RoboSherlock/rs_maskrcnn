# docker build -t ubuntu1604py3#6
FROM ubuntu:16.04
FROM waleedka/modern-deep-learning:latest

RUN rm -rf /var/lib/apt/lists/* \
    /etc/apt/sources.list.d/cuda.list \
    /etc/apt/sources.list.d/nvidia-ml.list

RUN apt-get update && \
    apt-get install -y software-properties-common vim
    
RUN apt-get update -y

# ENV variables for python3 - see http://click.pocoo.org/5/python3/
ENV LC_ALL C.UTF-8
ENV LANG C.UTF-8

# Silence Tensorflow warnings - look into compiling for CPU supported instruction sets
ENV TF_CPP_MIN_LOG_LEVEL 2



# ==================================================================
# tools
# ------------------------------------------------------------------    
RUN apt-get install -y libsm6 libxext6 libxrender-dev
RUN apt-get install -y python-tk
RUN apt-get install -y python-pip

RUN pip2 install scipy==0.19.1 \
	         keras==2.1.2 \
		 numpy==1.13.3 \
		 markdown==3.1.1 \	
		 setuptools==44.1.1 \
		 h5py==2.7.0 \
		 tensorflow \
		 PyWavelets==0.5.1 \
		 networkx==1.11 \
		 decorator==4.1.2 \
		 kiwisolver==1.1.0 \
		 matplotlib==2.2.3 \
		 Pillow==5.1.0 \
		 scikit-image==0.13.0 \
		 scikit-learn==0.19.1 \
	 	 mock==3.0.5 \
		 traitlets==4.3.3 \
 		 ipython==5.10.0 


# ==================================================================
# ROS
# ------------------------------------------------------------------ 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && apt-get install -y --allow-unauthenticated ros-kinetic-desktop && \
    apt-get install -y --allow-unauthenticated python-rosinstall python-rosdep python-vcstools python-catkin-tools && \
    apt-get install -y --allow-unauthenticated ros-kinetic-tf2-geometry-msgs
    
# bootstrap rosdep
RUN rosdep init
RUN rosdep update

    
# ==================================================================
# config & cleanup
# ------------------------------------------------------------------
RUN ldconfig && \
    rm -rf /var/lib/apt/lists/* /tmp/* ~/* 

# ==================================================================
# MKDIR
# ------------------------------------------------------------------

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN mkdir -p /home/catkin_ws/src
WORKDIR /home/catkin_ws/src
#RUN /opt/ros/kinetic/bin/catkin_init_workspace
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace /home/catkin_ws/src'

RUN cd /home/catkin_ws/src
RUN git clone https://github.com/tpatten/mask_rcnn_ros
WORKDIR /home/catkin_ws
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /home/catkin_ws; catkin_make'
RUN echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc




