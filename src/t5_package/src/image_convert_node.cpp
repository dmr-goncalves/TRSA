#include <ros/ros.h>
#include "t5_package/image_convert.h"

int main( int argc, char** argv )
{
    ros::init( argc, argv, "image_convert" );

    image_convert convert;

    convert.run();

    return EXIT_SUCCESS;
}
