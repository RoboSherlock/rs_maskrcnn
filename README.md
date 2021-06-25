# rs_maskrcnn
A RoboSherlock package managing the connection an annotation for maskrcnn 

## Dependencies
The src folder of your catkin workspace must contain the following packages:
* RoboSherlock: [https://github.com/RoboSherlock/robosherlock](https://github.com/RoboSherlock/robosherlock)
* mask_rcnn_ros_msgs Msgs: [https://github.com/sunava/mask_rcnn_msgs.git](https://github.com/sunava/mask_rcnn_msgs.git)

## Setup
* Clone this repository to the src folder of your workspace
* Open a terminal in your workspace folder and use `catkin build` (do **NOT** use `catkin_make`)
* Source your workspace
* For the provided dockerfile use: `sudo docker build -t rs_maskrcnn/docker .`

## Run
* Start a `roscore`
* To launch docker: `docker run --rm -d --network maskrcnn-docker-network --name rs_maskrcnn/docker `
* In docker: `rosrun mask_rcnn_ros mask_rcnn_service.py`
*  In your sourced workspace folder, type `rosrun robosherlock runAAE _ae:=mongo_maskrcnn_example _vis:=true`
    * The `_vis` tag is optional, if you like to see what the robot sees on your machine
    * The `_ae` tag however is crucial. It specifies the pipeline processing the vision. 

## Disclaimer
* This is only an example how to use the mask_rcnn_ros repository [https://github.com/tpatten/mask_rcnn_ros](https://github.com/tpatten/mask_rcnn_ros) within roboherlock
* the model is not trained on our database yet
* ![annotator resul](https://github.com/sunava/rs_maskrcnn/blob/main/maskrcnnannotator.png)
