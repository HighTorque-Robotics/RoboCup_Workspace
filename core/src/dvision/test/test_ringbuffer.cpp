#include "dvision/ringbuffer.hpp"
#include <gtest/gtest.h>
#include <thread>

using std::string;
using std::thread;
using std::to_string;
using dvision::RingBuffer;

struct Buffer {
    int idx;
};

RingBuffer<Buffer> ringbuffer(3, 1);

const int T = 5;
void consumer_thread() {
    ros::Rate r(5);
    for (int i = 0; i < T; i++) {
        auto& buf = ringbuffer.UserRequest();
        EXPECT_EQ(buf.idx, i);
        ROS_INFO("%d", i);
        r.sleep();
        ringbuffer.UserRelease();
    }
}

void worker_thread() {
    ros::Rate r(30);
    for (int i = 0; i < T; i++) {
        auto& buf = ringbuffer.WorkerRequest();
        buf.idx = i;
        r.sleep();
        ringbuffer.WorkerRelease();
    }
}

TEST(test, ringbuffer) {
    thread t1(consumer_thread);
    thread t2(worker_thread);
    t1.join();
    t2.join();
}

int
main(int argc, char** argv)
{
    //ros::init(argc, argv, "localization");
    ros::Time::init();
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
