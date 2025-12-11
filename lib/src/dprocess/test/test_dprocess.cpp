#include "dprocess/dprocess.hpp"
#include <gtest/gtest.h>

using namespace dprocess;
using namespace std;

TEST(dprocess, main)
{
    class Foo : public DProcess<Foo>
    {
      public:
        Foo(int freq, bool r)
          : DProcess(freq, r)
        {
        }
        void tick() override
        {
            usleep(1000);
        }
    };

    Foo f(10, true);
    f.spinOnce();
    f.join();
}

int
main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}