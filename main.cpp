#include"CamHandlerNode.h"
#include <chrono>


int main(int argc, char** argv)
{
    std::ios::sync_with_stdio(false);
    ros::init(argc,argv,"dahua_stereo");
    
    CamHandlerNode node;

    ros::spin();
    
    return 0;
}