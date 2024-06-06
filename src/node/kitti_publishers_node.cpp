#include <chrono>

#include "ros2_kitti_publishers/kitti_publishers_node.hpp"



using namespace cv;
using namespace std::chrono_literals;

KittiPublishersNode::KittiPublishersNode()
: Node("publisher_node"), file_index_(0)
{

  publisher_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("osdar/point_cloud", 10);
  publisher_image_color_left_ = this->create_publisher<sensor_msgs::msg::Image>("osdar/image/color/left", 10);
  publisher_image_color_mid_ = this->create_publisher<sensor_msgs::msg::Image>("osdar/image/color/mid", 10);
  publisher_image_color_right_ = this->create_publisher<sensor_msgs::msg::Image>("osdar/image/color/right", 10);

  init_file_path();

  create_publishers_data_file_names();

  timer_ = create_wall_timer(
    100ms, std::bind(&KittiPublishersNode::on_timer_callback, this));
}

void KittiPublishersNode::on_timer_callback()
{
    // 01- KITTI POINT CLOUDS2 MESSAGES START//
    sensor_msgs::msg::PointCloud2 point_cloud2_msg;
    convert_pcl_to_pointcloud2(point_cloud2_msg);
    // 01- KITTI POINT CLOUDS2 MESSAGES END//
 
    auto image_message_color_left = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_color_left = path_image_color_left_ + file_names_image_color_left_[file_index_];
    convert_image_to_msg(*image_message_color_left, img_pat_color_left);

    auto image_message_color_mid = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_color_mid = path_image_color_mid_ + file_names_image_color_mid_[file_index_];
    convert_image_to_msg(*image_message_color_mid, img_pat_color_mid);

    auto image_message_color_right = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_color_right = path_image_color_right_ + file_names_image_color_right_[file_index_];
    convert_image_to_msg(*image_message_color_right, img_pat_color_right);
    // 02- KITTI IMAGE MESSAGES END // 

    publisher_point_cloud_->publish(point_cloud2_msg);
    publisher_image_color_left_->publish(std::move(image_message_color_left));
    publisher_image_color_mid_->publish(std::move(image_message_color_mid));
    publisher_image_color_right_->publish(std::move(image_message_color_right));

    file_index_++;
}

void KittiPublishersNode::convert_pcl_to_pointcloud2(sensor_msgs::msg::PointCloud2 & msg ){
    pcl::PointCloud<pcl::PointXYZI> cloud;

    std::string filePath = get_path(KittiPublishersNode::PublisherType::POINT_CLOUD) + file_names_point_cloud_[file_index_];
    std::fstream input(filePath, std::ios::in | std::ios::binary);
    if(!input.good()){
      RCLCPP_INFO(this->get_logger(), "Could not read the point cloud. Check your file path!");
      exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    for (int i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        cloud.push_back(point);
    }

    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "base_link";
    msg.header.stamp = now();
}

void KittiPublishersNode::init_file_path()
{
    path_point_cloud_ = "data/osdar23_station_klein_flottbek/pointcloud/";
    path_image_color_left_ = "data/osdar23_station_klein_flottbek/rgb_highres_left/";
    path_image_color_mid_ = "data/osdar23_station_klein_flottbek/rgb_highres_center/";
    path_image_color_right_ = "data/osdar23_station_klein_flottbek/rgb_highres_right/";
}

std::string KittiPublishersNode::get_path(KittiPublishersNode::PublisherType publisher_type)
{
  RCLCPP_INFO(this->get_logger(), "get_path: '%i'", static_cast<int>(publisher_type));
  std::string path;
  if (publisher_type == KittiPublishersNode::PublisherType::POINT_CLOUD){
    path = path_point_cloud_;
    // print the path to the console
    RCLCPP_INFO(this->get_logger(), "path_point_cloud_: '%s'", path_point_cloud_.c_str());
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_LEFT_COLOR){
    path = path_image_color_left_;
    RCLCPP_INFO(this->get_logger(), "path_image_color_left_: '%s'", path_image_color_left_.c_str());
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_MID_COLOR){
    path = path_image_color_mid_;
    RCLCPP_INFO(this->get_logger(), "path_image_color_mid_: '%s'", path_image_color_mid_.c_str());
  }else{

    // Make sure that the publisher type is IMAGE_RIGHT_COLOR and otherwise raise an exception

    if (publisher_type != KittiPublishersNode::PublisherType::IMAGE_RIGHT_COLOR){
      throw std::invalid_argument("Invalid publisher type");
    } 
    path = path_image_color_right_;
    RCLCPP_INFO(this->get_logger(), "path_image_color_right_: '%s'", path_image_color_right_.c_str());
  }
  return path;
}

std::vector<std::string> KittiPublishersNode::get_filenames(PublisherType publisher_type)
{
  if (publisher_type == KittiPublishersNode::PublisherType::POINT_CLOUD){
     return file_names_point_cloud_;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_LEFT_COLOR){
     return file_names_image_color_left_;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_MID_COLOR){
     return file_names_image_color_mid_;
  }else{
    
    // Make sure that the publisher type is IMAGE_RIGHT_COLOR and otherwise raise an exception

    if (publisher_type != KittiPublishersNode::PublisherType::IMAGE_RIGHT_COLOR){
      throw std::invalid_argument("Invalid publisher type");
    }

    return file_names_image_color_right_;
  }
}

void KittiPublishersNode::set_filenames(PublisherType publisher_type, std::vector<std::string> file_names)
{
  if (publisher_type == KittiPublishersNode::PublisherType::POINT_CLOUD){
      file_names_point_cloud_= file_names;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_LEFT_COLOR){
      file_names_image_color_left_= file_names;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_MID_COLOR){
      file_names_image_color_mid_= file_names;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_RIGHT_COLOR){
      file_names_image_color_right_ = file_names;
  }
}

void KittiPublishersNode::create_publishers_data_file_names()
{
  for ( int type_index = 0; type_index != 4; type_index++ )
  {
    KittiPublishersNode::PublisherType type = static_cast<KittiPublishersNode::PublisherType>(type_index);
    std::vector<std::string> file_names = get_filenames(type);

   try
   {
      for (const auto & entry : std::filesystem::directory_iterator(get_path(type))){
        if (entry.is_regular_file()) {
            file_names.push_back(entry.path().filename());
        }
      }

      //Order lidar file names
      std::sort(file_names.begin(), file_names.end(),
            [](const auto& lhs, const auto& rhs) {
                return lhs  < rhs ;
            });
      set_filenames(type, file_names);
    }catch (const std::filesystem::filesystem_error& e)
    {
        RCLCPP_ERROR(this->get_logger(), "File path not found.");
    }
  }
}

//https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp#L278
void KittiPublishersNode::convert_image_to_msg(sensor_msgs::msg::Image & msg, const std::string path  )
{
  Mat frame;
  frame = imread(path);

  // Print the path to the console
  RCLCPP_INFO(this->get_logger(), "path: '%s'", path.c_str());

  if (frame.empty())                      // Check for invalid input
  {
    RCLCPP_ERROR(this->get_logger(), "Image does not exist. Check your files path!");
    rclcpp::shutdown();
  }

  msg.height = frame.rows;
  msg.width = frame.cols;
  std::string type = mat_type2encoding(frame.type());
  msg.encoding = type;
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
  msg.header.frame_id = "base_link";
  msg.header.stamp = this->now();
}

std::string KittiPublishersNode::mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

std::vector<std::string> KittiPublishersNode::parse_file_data_into_string_array(std::string file_name, std::string delimiter)
{
    std::ifstream f(file_name.c_str()); //taking file as inputstream

    if(!f.good()){
      RCLCPP_INFO(this->get_logger(), "Could not read OXTS data. Check your file path!");
      exit(EXIT_FAILURE);
    }

    std::string file_content_string;
    if(f) {
        std::ostringstream ss;
        ss << f.rdbuf(); // reading data
        file_content_string = ss.str();
    }

    //https://www.codegrepper.com/code-examples/whatever/c%2B%2B+how+to+tokenize+a+string  
    std::vector<std::string> tokens;
    size_t first = 0;
    while(first < file_content_string.size()){
        size_t second = file_content_string.find_first_of(delimiter,first);
        //first has index of start of token
        //second has index of end of token + 1;
        if(second == std::string::npos){
            second = file_content_string.size();
        }
        std::string token = file_content_string.substr(first, second-first);
        tokens.push_back(token);
        first = second + 1;
    }

    return tokens;
}
