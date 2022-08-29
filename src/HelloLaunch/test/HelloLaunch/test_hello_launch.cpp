#include <gtest/gtest.h>

class TestSample: public ::testing::Test
{
public:
  TestSample(): a(1), b(2) {}

  void SetUp()
  {
    a = 3;
  }

protected:
  int64_t a, b;  // no meaning
};

TEST_F(TestSample, test_a) {
  EXPECT_EQ(a, 1); // doesn't pass, but OK as it is a sample
}

TEST_F(TestSample, test_b) {
  EXPECT_EQ(b, 2); // doesn't pass, but OK as it is a sample
}
