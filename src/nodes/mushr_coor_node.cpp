#include "mushr_coordination/mushr_coordination.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "mushr_coordination");
    ros::NodeHandle nh;

    node_mushr_coor::MushrCoordination node(nh);

    ros::spin();
    return 0;
}