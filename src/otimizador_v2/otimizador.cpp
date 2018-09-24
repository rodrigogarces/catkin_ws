#include "otimizador.h"

TargetGenerator::TargetGenerator(ros::NodeHandle *nodehandle):nh_(*nodehandle)
{
    ROS_INFO("Inicializando nó TargetGenerator");
    initializeSubscribers();
    initializePublishers();
}

void TargetGenerator::initializeSubscribers()
{
    ROS_INFO("inicializando os subscribers");
    map_sub_ = n.subscribe("/map", 1, mapCallback);                                             // Subscreve no topico que recebe o OG do ambiente
    curr_pose_sub_ = n.subscribe("/vrep_ros_interface/pose", 1, currPoseCallback);              // Subscreve tópico que recebe a posição corrente do robô
    costmap_sub_ = n.subscribe("/move_base_node/global_costmap/costmap", 1, costmapCallback);
}

void TargetGenerator::initializePublishers()
{
    ROS_INFO("inicializando os publishers");
    path_pub_ = n.advertise<std_msgs::String>("/path_plan", 1, true);                           // Publica o caminho para o alvo
    target_pub_ = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);   // Publica o caminho para o alvo
    finalize_pub_ = n.advertise<std_msgs::String>("/finalize_mapping", 1, true);                // Publica flag que para o mapeamento
}

void TargetGenerator::initializeServices()
{

}

