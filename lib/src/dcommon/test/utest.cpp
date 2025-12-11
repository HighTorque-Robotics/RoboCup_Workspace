#include "dcommon/util.hpp"
#include <gtest/gtest.h>

using namespace dcommon;

TEST(TestUtil, testStirng) {
  std::string foo("/foo/bar/test.cpp");
  EXPECT_EQ("/foo/bar/", util::dirname(foo));

  std::vector<std::string> bar;
  bar = util::string_split(foo, '/');
  bar.clear();
  util::string_split(foo, '/', bar);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

