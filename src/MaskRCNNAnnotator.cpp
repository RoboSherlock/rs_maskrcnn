/**
 * Copyright 2021 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Vanessa Hassouna <hassouna@uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <uima/api.hpp>

#include <pcl/point_types.h>
#include "ros/ros.h"
#include <cstdlib>

#include <mask_rcnn_msgs/MaskRcnn.h>
#include <mask_rcnn_msgs/Result.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_painter2D.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
//RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/DrawingAnnotator.h>
#include <pcl/visualization/image_viewer.h>


using namespace uima;
using namespace std;

class MaskRCNNAnnotator : public DrawingAnnotator {
private:
    cv::Mat color;

public:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb;

    MaskRCNNAnnotator() : DrawingAnnotator(__func__) {
    }

    TyErrorId initialize(AnnotatorContext &ctx) {
        outInfo("initialize");
        return UIMA_ERR_NONE;
    }

    TyErrorId destroy() {
        outInfo("destroy");
        return UIMA_ERR_NONE;
    }

    TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec) {
        outInfo("process start");
        rs::StopWatch clock;
        rs::SceneCas cas(tcas);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
        cas.get(VIEW_CLOUD, *cloud_ptr);
        cloud_rgb = cloud_ptr;
//        sensor_msgs::Image image;
//        toROSMsg(*cloud_ptr, image);

        cas.get(VIEW_COLOR_IMAGE, color);

        //transform opencv::mat color to sensor_msgs::Image
        cv_bridge::CvImage img_bridge;
        sensor_msgs::Image image; // >> message to be sent
        std_msgs::Header header; // empty header
        header.seq = 1; // user defined counter
        header.stamp = ros::Time::now(); // time
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, color);
        img_bridge.toImageMsg(image); // from cv_bridge to sensor_msgs::Image


        //make client for mask_rcnn_service
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<mask_rcnn_msgs::MaskRcnn>("mask_rcnn_service");
        mask_rcnn_msgs::MaskRcnn srv;
        srv.request.image = image;

        //call mask_rcnn_service with input image
        if (client.call(srv)) {
            mask_rcnn_msgs::MaskRcnn::Response response_msgs = srv.response;
            mask_rcnn_msgs::Result result = srv.response.result;

            int n = 1;
            int itbox = result.boxes.size();

            //iterate over all prediction
            for (int i = 0; i < itbox; i++) {


                int x = response_msgs.result.boxes[i].x_offset;
                int y = response_msgs.result.boxes[i].y_offset;
                int width = response_msgs.result.boxes[i].width;
                int height = response_msgs.result.boxes[i].height;

                // getting a rectangle for the boxes
                cv::Rect rect(x, y, width, height);

                cv::Scalar randomColor(
                        (double)std::rand() / RAND_MAX * 255,
                        (double)std::rand() / RAND_MAX * 255,
                        (double)std::rand() / RAND_MAX * 255);

                cv::rectangle(color, rect, randomColor, 4, 4);

                //drawing predicted class as text
                cv::putText(color,
                            response_msgs.result.class_names[i] + "" + std::to_string(response_msgs.result.scores[0]),
                            cv::Point(x,y), // Coordinates
                            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                            1.0, // Scale. 2.0 = 2x bigger
                            cv::Scalar(255,255,255), // BGR Color
                            1, // Line Thickness (Optional)
                            cv:: LINE_AA); // Anti-alias (Optional, see version note)


                //getting the mask as image from results and converting them into cv::mat
                sensor_msgs::Image instance_mask;
                instance_mask = response_msgs.result.masks[i];
                cv::Mat instance_mask_mat;
                instance_mask_mat = cv_bridge::toCvCopy(instance_mask, "rgb8")->image;
                double opacity = 0.4;

                //adding mask to our cv::mat display
                cv::addWeighted(instance_mask_mat, opacity, color, 1 - opacity, 0, color);

            }
        } else {
            ROS_ERROR("Failed to call service maskrcnn");
            return 1;
        }


        outInfo("Cloud size: " << cloud_ptr->points.size());
        outInfo("took: " << clock.getTime() << " ms.");
        return UIMA_ERR_NONE;
    }


    void drawImageWithLock(cv::Mat &disp) override {
        disp = color.clone();
    }


    void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
        double pointSize = 1.0;
        if (firstRun) {
            visualizer.addPointCloud(cloud_rgb, std::string("cloud"));
            visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize,
                                                        std::string("cloud"));
        } else {
            visualizer.updatePointCloud(cloud_rgb, std::string("cloud"));
            visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize,
                                                        std::string("cloud"));
        }
    }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(MaskRCNNAnnotator)