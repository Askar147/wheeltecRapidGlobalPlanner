/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <iostream>
#include <vector>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h> // For read/write/close
#include <chrono>
#include <thread>

#include <chrono>
using namespace std::chrono;

#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <global_planner/dijkstra.h>
#include <global_planner/astar.h>
#include <global_planner/grid_path.h>
#include <global_planner/gradient_path.h>
#include <global_planner/quadratic_calculator.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {

void GlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

GlobalPlanner::GlobalPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
    //initialize the planner
    initialize(name, costmap, frame_id);
}

GlobalPlanner::~GlobalPlanner() {
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
    if (dsrv_)
        delete dsrv_;
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        bool use_quadratic;
        private_nh.param("use_quadratic", use_quadratic, true);
        if (use_quadratic)
            p_calc_ = new QuadraticCalculator(cx, cy);
        else
            p_calc_ = new PotentialCalculator(cx, cy);

        bool use_dijkstra;
        private_nh.param("use_dijkstra", use_dijkstra, true);
        if (use_dijkstra)
        {
            DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
            if(!old_navfn_behavior_)
                de->setPreciseStart(true);
            planner_ = de;
        }
        else
            planner_ = new AStarExpansion(p_calc_, cx, cy);

        bool use_grid_path;
        private_nh.param("use_grid_path", use_grid_path, false);
        if (use_grid_path)
            path_maker_ = new GridPath(p_calc_);
        else
            path_maker_ = new GradientPath(p_calc_);

        orientation_filter_ = new OrientationFilter();

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);
        private_nh.param("outline_map", outline_map_, true);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlanner::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>::CallbackType cb = boost::bind(
                &GlobalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}

void GlobalPlanner::reconfigureCB(global_planner::GlobalPlannerConfig& config, uint32_t level) {
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
}

void GlobalPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool GlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    if (start.header.frame_id != global_frame) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    //std::string out1 = "NX = " + std::to_string(nx) + " NY = " + std::to_string(ny); 
    ROS_WARN("NX = %d NY = %d", nx, ny);
    
    if(outline_map_)
        outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    const char* serverIp = "127.0.0.1";
    int port = 12345;
    bool found_legal;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    found_legal = GlobalPlanner::sendDataToJava(serverIp, port, costmap_->getCharMap(), costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY(), start_x, start_y, goal_x, goal_y, nx * ny * 2);
    
    //found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y, nx * ny * 2, potential_array_);
    

    std::cout << "RECIVED POTENTIAL ARRAY IN MAIN" <<std::endl;
 //   for (int i = 0; i < 416*160; i++) {
 //        if (potential_array_[i] != 1e+10){
 //            std::cout << potential_array_[i] << " ";
 //   }
  //  }

    if(!old_navfn_behavior_)
        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
    if(publish_potential_)
        publishPotential(potential_array_);
    //TODO: does not return from SendDATA TO JAVA, return found_legal instead of true
    if (found_legal) {
        //extract the plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        } else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
        ROS_ERROR("Failed to get a plan.");
    }

    // add orientations if needed
    orientation_filter_->processPath(start, plan);

    //publish the plan for visualization purposes
    publishPlan(plan);
    delete potential_array_;
    return !plan.empty();
}

void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool GlobalPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}

void GlobalPlanner::publishPotential(float* potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else
            grid.data[i] = potential_array_[i] * publish_scale_ / max;
    }
    potential_pub_.publish(grid);
}

bool isLittleEndian() {
    uint16_t x = 1; // 0x0001
    auto ptr = reinterpret_cast<char*>(&x);
    return ptr[0] == 1; // Little endian if true
}

// Custom htonll implementation
uint64_t htonll(uint64_t value) {
    // Check if we're on a little-endian system; no change needed for big-endian
    if (isLittleEndian()) {
        // Swap bytes
        const uint32_t high_part = htonl(static_cast<uint32_t>(value >> 32));
        const uint32_t low_part = htonl(static_cast<uint32_t>(value & 0xFFFFFFFFLL));
        
        // Combine parts into final value
        return (static_cast<uint64_t>(low_part) << 32) | high_part;
    } else {
        return value;
    }
}

// Helper function to convert double to network byte order
std::vector<unsigned char> doubleToBytes(double value) {
    std::vector<unsigned char> bytes(sizeof(double));
    uint64_t netValue = htonll(*reinterpret_cast<uint64_t*>(&value));
    std::memcpy(bytes.data(), &netValue, sizeof(double));
    return bytes;
}

ssize_t readFully(int sock, void* buf, size_t len) {
    size_t total = 0; // how many bytes we've read
    size_t bytesleft = len; // how many we have left to read
    ssize_t n;

    while(total < len) {
        n = read(sock, (char*)buf + total, bytesleft);
        if (n == -1) { break; } // Error
        if (n == 0) { break; } // Connection closed
        total += n;
        bytesleft -= n;
    }

    return n == -1 ? -1 : total; // return -1 on failure, total on success
}

struct Tuple {
    float floatValue;
    int intValue;
};

// Function to send data to Java server
bool GlobalPlanner::sendDataToJava(const char* serverIp, int port, const unsigned char* costs, size_t costsSize, double s_x, double s_y, double end_x, double end_y, int cycles) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return false;
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    inet_pton(AF_INET, serverIp, &serverAddr.sin_addr);

    if (connect(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Connection Failed" << std::endl;
        return false;
    }
    
//    for (int i = 0; i < 416*160; i++) {
//        float a = costs[i];
//	std::cout<<a<<std::endl;
//    }

    // Serialize data
    std::vector<unsigned char> buffer;
    auto appendDouble = [&](double value) {
        auto bytes = doubleToBytes(value);
        buffer.insert(buffer.end(), bytes.begin(), bytes.end());
    };

    appendDouble(s_x);
    appendDouble(s_y);
    appendDouble(end_x);
    appendDouble(end_y);

    int netCycles = htonl(cycles);
    int netSize = htonl(costsSize);
    buffer.insert(buffer.end(), reinterpret_cast<unsigned char*>(&netCycles), reinterpret_cast<unsigned char*>(&netCycles) + sizeof(netCycles));
    buffer.insert(buffer.end(), reinterpret_cast<unsigned char*>(&netSize), reinterpret_cast<unsigned char*>(&netSize) + sizeof(netSize));
    buffer.insert(buffer.end(), costs, costs + costsSize);

    // Send data
    send(sock, buffer.data(), buffer.size(), 0);

    // Read the response from the Java server
    bool foundLegal;
    readFully(sock, &foundLegal, sizeof(foundLegal));

//    int byteArraySize;
//    readFully(sock, &byteArraySize, sizeof(byteArraySize));
//    byteArraySize = ntohl(byteArraySize); // Convert from network to host byte order

//    std::vector<unsigned char> byteArray(byteArraySize);
//    readFully(sock, byteArray.data(), byteArraySize);

    // Assuming the system requires conversion from big-endian (network order)
//    std::vector<float> potentialArray(byteArraySize / sizeof(float));
//    for (size_t i = 0; i < potentialArray.size(); ++i) {
//        uint32_t temp;
//        std::memcpy(&temp, &byteArray[i * sizeof(float)], sizeof(float));
 //       temp = ntohl(temp); // Convert each float from network byte order
//        std::memcpy(&potentialArray[i], &temp, sizeof(float));
 //   }

    // Print the received potential array
 //   std::cout << "Received potential array (" << potentialArray.size() << " elements):" << std::endl;
//     for (size_t i = 0; i < potentialArray.size(); ++i) {
//         if (potentialArray[i] != 1e+10){
//            std::cout << potentialArray[i] << " ";
//    }
//    }
    std::cout << std::endl;

    int length;
    readFully(sock, &length, sizeof(length));
    length = ntohl(length); // Convert from network byte order to host byte order

    // Allocate space for the tuples and read them
    std::vector<Tuple> tuples(length);
    for (int i = 0; i < length; ++i) {
        // Read the float part of the tuple
        uint32_t tempFloat;
        readFully(sock, &tempFloat, sizeof(tempFloat));
        tempFloat = ntohl(tempFloat); // Convert to host byte order
        memcpy(&tuples[i].floatValue, &tempFloat, sizeof(tempFloat));

        // Read the int part of the tuple
        readFully(sock, &tuples[i].intValue, sizeof(tuples[i].intValue));
        tuples[i].intValue = ntohl(tuples[i].intValue); // Convert to host byte order
    }

   // for (const Tuple& tuple : tuples) {
       // std::cout << "Received tuple: (" << tuple.floatValue << ", " << tuple.intValue << ")" << std::endl;
    //}



    // Close socket
    close(sock);
    std::cout << "GOT BEFORE POTENTIAL ARRAY END" << std::endl;
    this->potential_array_ = new float[416*160];
    std::fill(potential_array_, potential_array_ + 416*160, POT_HIGH);
    std::cout << "GOT BETWEEN POTENTIAL ARRAY END" << std::endl;
    for (int i = 0; i < tuples.size(); i++) {
        potential_array_[tuples[i].intValue] = tuples[i].floatValue;
    }
    std::cout << "GOT TO THE POTENTIAL ARRAY END" << std::endl;
    return foundLegal;
}

} //end namespace global_planner
