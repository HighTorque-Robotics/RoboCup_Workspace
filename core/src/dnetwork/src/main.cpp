/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file main.cpp
 * @brief
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-24
 */

#include "dnetwork/gamecontroller.hpp"
#include "dnetwork/team.hpp"

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "dnetwork_node");
    ros::NodeHandle nh("~");

    dnetwork::Team t(&nh);
    dnetwork::GameController g(&nh);

    t.spin();
    g.spin();

    t.join();
    g.join();
}
