#define BOOST_TEST_MODULE My first test module
// #define BOOST_TEST_NO_MAIN
// #define BOOST_TEST_ALTERNATIVE_INIT_API
#include <boost/test/included/unit_test.hpp>
#include <iostream>
#include <list>
#include <vector>

BOOST_AUTO_TEST_CASE(first_test_fuction) {
    int a = 42;

    BOOST_TEST(a > 0);
}

class point3d {
    int x_;
    int y_;
    int z_;

public:
    point3d(int const x = 0, int const y = 0, int const z = 0)
        : x_(x), y_(y), z_(z) {}

    int x() const {
        return x_;
    }
    point3d& x(int const x) {
        x_ = x;
        return *this;
    }
    int y() const {
        return y_;
    }
    point3d& y(int const y) {
        y_ = y;
        return *this;
    }
    int z() const {
        return z_;
    }
    point3d& z(int const z) {
        z_ = z;
        return *this;
    }

    bool operator==(point3d const& pt) const {
        return x_ == pt.x_ && y_ == pt.y_ && z_ == pt.z_;
    }
    bool operator!=(point3d const& pt) const {
        return !(*this == pt);
    }
    bool operator<(point3d const& pt) const {
        return x_ < pt.x_ || y_ < pt.y_ || z_ < pt.z_;
    }
    friend std::ostream& operator<<(std::ostream& stream, point3d const& pt) {
        stream << "(" << pt.x_ << ", " << pt.y_ << ", " << pt.z_ << ")";
        return stream;
    }
    void offset(int const offsetx, int const offsety, int const offsetz) {
        x_ += offsetx;
        y_ += offsety;
        z_ += offsetz;
    }
    static point3d origin() {
        return point3d{};
    }
};

// int main(int argc, char* argv[]) {
//     return boost::unit_test::unit_test_main(init_unit_test, argc, argv);
// }

BOOST_AUTO_TEST_SUITE(test_construction)

BOOST_AUTO_TEST_CASE(test_constructor) {
    auto p = point3d{1, 2, 3};
    BOOST_TEST(p.x() == 1);
    BOOST_TEST(p.y() == 2);
    BOOST_TEST(p.z() == 4); // will fail
}

BOOST_AUTO_TEST_CASE(test_origin) {
    auto p = point3d::origin();
    BOOST_TEST(p.x() == 0);
    BOOST_TEST(p.y() == 0);
    BOOST_TEST(p.z() == 0);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(test_operations) // nested test suite
BOOST_AUTO_TEST_SUITE(test_methods)

BOOST_AUTO_TEST_CASE(test_offset) {
    auto p = point3d{1, 2, 3};
    p.offset(1, 1, 1);
    BOOST_TEST(p.x() == 2);
    BOOST_TEST(p.y() == 3);
    BOOST_TEST(p.z() == 3); // will fail
}

BOOST_AUTO_TEST_SUITE_END()
BOOST_AUTO_TEST_SUITE_END()

// add decorators: description, label, precondition, dependency, fixture
BOOST_AUTO_TEST_SUITE(test_operations)
BOOST_AUTO_TEST_SUITE(test_operators)

BOOST_AUTO_TEST_CASE(test_equal,
                     *boost::unit_test::description("test operator==") *
                         boost::unit_test::label("opeq")) {
    auto p1 = point3d{1, 2, 3};
    auto p2 = point3d{1, 2, 3};
    auto p3 = point3d{3, 2, 1};
    BOOST_TEST(p1 == p2);
    BOOST_TEST(p1 == p3); // will fail
}

BOOST_AUTO_TEST_CASE(test_not_equal,
                     *boost::unit_test::description("test operator!=") *
                         boost::unit_test::label("opeq") *
                         boost::unit_test::depends_on(
                             "test_operations/test_operators/test_equal")) {
    auto p1 = point3d{1, 2, 3};
    auto p2 = point3d{3, 2, 1};
    BOOST_TEST(p1 != p2);
}

BOOST_AUTO_TEST_CASE(test_less) {
    auto p1 = point3d{1, 2, 3};
    auto p2 = point3d{1, 2, 3};
    auto p3 = point3d{3, 2, 1};
    BOOST_TEST(!(p1 < p2));
    BOOST_TEST(p1 < p3);
}

BOOST_AUTO_TEST_SUITE_END()
BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(test_assertion)

BOOST_AUTO_TEST_CASE(test_basic) {
    double a1 = 4.201, a2 = 4.200;
    BOOST_TEST(a1 == a2, boost::test_tools::tolerance(1e-3));

    std::vector<int> v{1, 2, 3};
    std::list<short> l{1, 2, 3};
    BOOST_TEST(v == l, boost::test_tools::per_element());

    int b = 1;
    BOOST_TEST((b > 0 ? true : false));
}

BOOST_AUTO_TEST_SUITE_END()

// Test fixture
struct standard_fixture {
    standard_fixture() {
        BOOST_TEST_MESSAGE("setup");
    }
    ~standard_fixture() {
        BOOST_TEST_MESSAGE("cleanup");
    }
    int n{42};
};
struct extended_fixture {
    std::string name;
    int         data;
    extended_fixture(std::string const& n = "") : name(n), data(0) {
        BOOST_TEST_MESSAGE("setup " + name);
    }
    ~extended_fixture() {
        BOOST_TEST_MESSAGE("cleanup " + name);
    }
};

void fixture_setup() {
    BOOST_TEST_MESSAGE("fixture setup");
}
void fixture_cleanup() {
    BOOST_TEST_MESSAGE("fixture cleanup");
}

BOOST_FIXTURE_TEST_CASE(test_case, extended_fixture) {
    data++;
    BOOST_TEST(data == 1);
}

BOOST_FIXTURE_TEST_SUITE(suite1, extended_fixture)
BOOST_AUTO_TEST_CASE(case1) {
    BOOST_TEST(data == 0);
}
BOOST_AUTO_TEST_CASE(case2) {
    data++;
    BOOST_TEST(data == 1);
}

BOOST_FIXTURE_TEST_CASE(case3, standard_fixture) {
    BOOST_TEST(n == 42);
}

BOOST_AUTO_TEST_SUITE_END()

// define more than a single fixture
BOOST_AUTO_TEST_CASE(
    test_case_multifix,
    *boost::unit_test::fixture<extended_fixture>(std::string("fix1")) *
        boost::unit_test::fixture<extended_fixture>(std::string("fix2")) *
        boost::unit_test::fixture<standard_fixture>()) {
    BOOST_TEST(true);
}

// use free functiona s setup and teardown
BOOST_AUTO_TEST_CASE(test_case_multifix_freefunc,
                     *boost::unit_test::fixture(&fixture_setup,
                                                &fixture_cleanup)) {
    BOOST_TEST(true);
}