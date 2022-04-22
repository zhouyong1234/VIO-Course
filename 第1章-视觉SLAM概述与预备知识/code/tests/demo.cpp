#include <gtest/gtest.h>
#include <iostream>


int add(int a, int b)
{
    return a + b;
}

int sub(int a, int b)
{
    return a - b;
}


TEST(TestAddInt, test_add_int_1) {
  int res = add(10, 24);
  EXPECT_EQ(res, 34);
}

TEST(TestSubInt, test_sub_int_1) {
  int res = sub(40, 96);
  EXPECT_EQ(res, -56);
}


int main(int argc, char **argv)
{
    // std::cout << "google test" << std::endl;
    testing::InitGoogleTest(&argc, argv);
    // EXPECT_EQ(add(1, 1), 2);
    // EXPECT_EQ(add(1, 1), 1) << "FAILED: EXPECT: 2, but given 1";
    // return 0;
    return RUN_ALL_TESTS();
}