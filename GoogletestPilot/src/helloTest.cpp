#include <gtest/gtest.h>
#include <numeric>

TEST(HelloTestSuite, BasicTest) {
  EXPECT_TRUE(2+2 == 2*2);
  EXPECT_FALSE(1 == 2);
  ASSERT_TRUE(2+2 == 2*2);
  ASSERT_FALSE(1 == 2);

  auto a = 42, b = 10;
  ASSERT_TRUE(a > 0);
  EXPECT_EQ(a, 42);
  EXPECT_NE(a, b);
  EXPECT_LT(b, a);
  EXPECT_LE(b, 11);
  EXPECT_GT(a, b);
  EXPECT_GE(b, 10);
}

// Demonstrate some basic assertions.
TEST(HelloTestSuite, BasicTestString) {
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  auto str = "sample";
  EXPECT_STREQ(str, "sample");
  EXPECT_STRNE(str, "simple");
  ASSERT_STRCASEEQ(str, "SAMPLe");
  ASSERT_STRCASENE(str, "SIMPLE");
}

TEST(HelloTestSuite, BasicTestFloat) {
  float a = 1.9999999f, b = 1.9999998f;
  EXPECT_FLOAT_EQ(a, b);
  ASSERT_FLOAT_EQ(a, b);

  ASSERT_NEAR(a, b, 1e-5);

  // ASSERT_DOUBLE_EQ(a, b);
}



class point3d {
  int x_;
  int y_;
  int z_;
public:
  point3d(int const x = 0, int const y = 0, int const z = 0) : x_(x), y_(y), z_(z) {}
  int x() const {
    return x_;
  }
  int y() const {
    return y_;
  }
  int z() const {
    return z_;
  }

  point3d &x (int const nx) {
    x_ = nx;
    return *this;
  }
  point3d &y (int const ny) {
    y_ = ny;
    return *this;
  }
  point3d &z (int const nz) {
    z_ = nz;
    return *this;
  }

  void offset(int dx, int dy, int dz) {
    x_ += dx;
    y_ += dy;
    z_ += dz;
  }

  static point3d origin() { return point3d(); }
};

TEST(/*DISABLED_*/TestConstruction, TestConstructor) {
  auto p = point3d{1, 2, 3};
  ASSERT_EQ(p.x(), 1);
  ASSERT_EQ(p.y(), 2);
  ASSERT_EQ(p.z(), 3);
}

TEST(TestConstruction, TestOrigin) {
  auto p = point3d::origin();
  ASSERT_EQ(p.x(), 0);
  ASSERT_EQ(p.y(), 0);
  ASSERT_EQ(p.z(), 0);
}

TEST(TestConstruction, TestSet) {
  auto p = point3d::origin();
  p.x(3).y(4).z(5);
  ASSERT_EQ(p.x(), 3);
  ASSERT_EQ(p.y(), 4);
  ASSERT_EQ(p.z(), 5);
}


void function_that_throws() {
  throw std::runtime_error("error");
}
void function_no_throw() {
}

TEST(TestAssertions, Exceptions) {
  EXPECT_THROW(function_that_throws(), std::runtime_error);
  EXPECT_ANY_THROW(function_that_throws());
  EXPECT_NO_THROW(function_no_throw());

  ASSERT_THROW(function_that_throws(), std::runtime_error);
  ASSERT_ANY_THROW(function_that_throws());
  ASSERT_NO_THROW(function_no_throw());
}

bool is_positive(int const val) {
  return val > 0;
}
bool is_double(int const val1, int const val2) {
  return val2 + val2 == val1;
}

TEST(TestAssertions, Predicates) {
  EXPECT_PRED1(is_positive, 42);
  EXPECT_PRED2(is_double, 42, 21);

  ASSERT_PRED1(is_positive, 42);
  ASSERT_PRED2(is_double, 42, 21);
}

/// test fixtures
class TestFixture : public ::testing::Test {
protected:
  TestFixture() {
    std::cout << "constructing fixture\n";
    data.resize(10);
    std::iota(std::begin(data), std::end(data), 1);
  }
  ~TestFixture() {
    std::cout << "destroying fixture\n";
  }

protected:
  std::vector<int> data;
};

TEST_F(TestFixture, TestData) {
  ASSERT_EQ(data.size(), 10);
  ASSERT_EQ(data[0], 1);
  ASSERT_EQ(data[data.size()-1], data.size());
}

// TODO: customize testing environment

// w/o this main, it runs; when linking to gtest_main shared lib
// int main(int argc, char* argv[]) {
//   ::testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }