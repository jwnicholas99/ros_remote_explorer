#include <ros/ros.h>

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <slam/GridImage.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <vector>

#if CV_VERSION_MAJOR > 3
#include <opencv2/imgcodecs/legacy/constants_c.h>
#endif

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

class GridImageNode {
    public:
        GridImageNode(){
            // Need to create a publisher for GridImg.msg, not this
            grid_image_pub_ = n_.advertise<slam::GridImage>("grid_image", 1);
            map_sub_ = n_.subscribe("map", 1, &GridImageNode::gridImageCallback, this);
        
            ROS_INFO("GridImageNode started.");
        }

        void gridImageCallback(const nav_msgs::OccupancyGrid& map){
            int width = map.info.width;
            int height = map.info.height;
            map_mat = cv::Mat(height, width, CV_8U);

            const std::vector<int8_t>& map_data (map.data);

            unsigned char *map_mat_data_p=(unsigned char*) map_mat.data;

            // Convert map to image
            // We have to flip around the y axis, y for image starts at the top and y for map at the bottom
            int height_rev = height-1;
            for (int y = height_rev; y >= 0; --y){
                int idx_map_y = width * (height - y);
                int idx_img_y = width * y;

                for (int x=0; x < width; ++x){
                    int idx = idx_img_y + x;
                    
                    switch (map_data[idx_map_y + x]){
                        case -1:
                            map_mat_data_p[idx] = 127;
                            break;
                        
                        case 0:
                            map_mat_data_p[idx] = 255;
                            break;

                        case 100:
                            map_mat_data_p[idx] = 0;
                            break;
                    }
                }
            }

            // Compress image
            // Reference: compressed_image_transport's compressed_publisher.cpp
            sensor_msgs::CompressedImage compressed_msg;
            compressed_msg.format = "mono8";

            std::vector<int> params;
            params.resize(9,0);
            params[0] = IMWRITE_JPEG_QUALITY;
            params[1] = 80;
            params[2] = IMWRITE_JPEG_PROGRESSIVE;
            params[3] = 0;
            params[4] = IMWRITE_JPEG_OPTIMIZE;
            params[5] = 0;
            params[6] = IMWRITE_JPEG_RST_INTERVAL;
            params[7] = 0;

            int bitDepth = enc::bitDepth(compressed_msg.format);
            int numChannels = enc::numChannels(compressed_msg.format);

            // Start compression
            compressed_msg.format += "; jpeg compressed ";
            if ((bitDepth == 8) || (bitDepth == 16)){
                std::string targetFormat;
                if (enc::isColor(compressed_msg.format)){
                    // convert color images to BGR8 format
                    targetFormat = "bgr8";
                    compressed_msg.format += targetFormat;
                }

                // Compress using OpenCV
                if(cv::imencode(".jpg", map_mat, compressed_msg.data, params)){
                    float cRatio = (float)(map_mat.rows * map_mat.cols * map_mat.elemSize())
                        / (float)compressed_msg.data.size();
                    ROS_INFO("Compressed image - Codec: jpg, Compression Ratio: 1:%.2f (%lu bytes)", cRatio, compressed_msg.data.size());
                } else {
                    ROS_ERROR("cv::imencode (jpeg) failed on input image");
                }
            }
            
            // Publish GridImage msg
            slam::GridImage msg;
            msg.info = map.info;
            msg.image = compressed_msg;
            grid_image_pub_.publish(msg);
        }

        ros::Subscriber map_sub_;
        ros::Publisher grid_image_pub_;
        cv::Mat map_mat;

        ros::NodeHandle n_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "grid_image_node");
    GridImageNode grid_image_node;
    ros::spin();

    return 0;
}