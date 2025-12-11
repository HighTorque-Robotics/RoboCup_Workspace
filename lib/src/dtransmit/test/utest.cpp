#include <gtest/gtest.h>
#include "dtransmit/dtransmit.hpp"

using namespace dtransmit;

TEST(TestDTransmit, testGetBroadcastIP) {
  DTransmit transmitter;

  int portBase = 26334;
  int NUM = 1000;
  struct Foo {
    int x = 1;
    double y = 2;
    std::string s = "hello";
  } foo;

  // test receiving
  transmitter.addRawRecv(portBase, [&](const void *buffer, std::size_t size) {
    if (size != sizeof(foo)) {
      throw std::runtime_error("Received raw size not correct");
    }
    Foo *f = (Foo *) buffer;
    std::cout << buffer << std::endl;
    printf("recv: %d %lf %s", f->x, f->y, f->s.c_str());
  });

  transmitter.startService();

  // test sending
  for (int i = 0; i < NUM; ++i) {
    transmitter.sendRaw(portBase, (void *) &foo, sizeof(foo));
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
