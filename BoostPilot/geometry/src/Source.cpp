#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/ring.hpp>

#include <boost/geometry/geometries/adapted/c_array.hpp>
BOOST_GEOMETRY_REGISTER_C_ARRAY_CS(boost::geometry::cs::cartesian)
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(boost::geometry::cs::cartesian)

#include <boost/geometry/index/rtree.hpp>
#include <vector>
#include <algorithm>
#include <boost/foreach.hpp>
#include <functional>

// Boost.Geometry is header only. There is not linking with any library necessary

bool IsConvex(/*const*/ boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> &polygon)
{
    boost::geometry::correct(polygon);
    boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> hull;
    boost::geometry::convex_hull(polygon, hull);

    return boost::geometry::area(hull) == boost::geometry::area(polygon);
}

using Point3D = boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>;

std::ostream &operator<<(std::ostream &os, const Point3D &p)
{
    using boost::geometry::get;
    os << "(" << get<0>(p) << ", " << get<1>(p) << ", " << get<2>(p) << ")";
    return os;
}

std::function<bool(const Point3D& p1, const Point3D& p2)> OrderByZ() noexcept {
    return [](const Point3D& p1, const Point3D& p2) {
        using boost::geometry::get;
        if (get<2>(p1) < get<2>(p2)) return true;
        if (get<2>(p1) > get<2>(p2)) return false;
        if (get<1>(p1) < get<1>(p2)) return true;
        if (get<1>(p1) > get<1>(p2)) return false;
        return get<0>(p1) < get<0>(p2);
    };
}

int main()
{
    namespace bg = boost::geometry;
    namespace bgi = boost::geometry::index;

    /// Cartesian
    using Coordinate2D = bg::model::d2::point_xy<double, bg::cs::cartesian>;
    Coordinate2D p(0, 0), p1(1, 1), p2(2, 2);
    std::cout << "Distance p1-p2 is: " << bg::distance(p1, p2) << std::endl;
    bg::assign_point(p, p1);
    assert(bg::equals(p, p1));
    bg::add_point(p, p2);
    bg::subtract_point(p, p2);
    assert(bg::equals(p, p1));

    double a[2] = {1, 1};
    double b[2] = {2, 3};
    auto d = bg::distance(a, b);
    std::cout << "Distance with C array a-b is: " << d << std::endl;
    std::cout << std::endl;

    /// Polygon from set of ponts
    // double points[][2] = {{2.0, 1.3}, {4.1, 3.0}, {5.3, 2.6}, {2.9, 0.7}, {2.0, 1.3}};
    std::vector<Coordinate2D> points{
        {2.0, 1.3}, {4.1, 3.0}, {5.3, 2.6}, {2.9, 0.7}, {2.0, 1.3}};
    bg::model::polygon<Coordinate2D> poly;
    boost::geometry::append(poly, points);
    boost::tuple<double, double> tp = boost::make_tuple(3.7, 2.0);
    std::cout << "Point p is in polygon? " << std::boolalpha << bg::within(tp, poly) << std::endl;
    std::cout << "Polygon area: " << bg::area(poly) << std::endl;
    // mix C array point with Boost.Tuple point
    std::cout << "Distance of mix point a-p is: " << bg::distance(a, tp) << std::endl;
    bg::model::ring<Coordinate2D> points_again;
    bg::convert(poly, points_again);
    // assert(poitns == points_again);
    assert(std::equal(points.begin(), points.end(), points_again.begin(), [](const Coordinate2D &a, const Coordinate2D &b)
                      { return bg::equals(a, b); }));
    std::cout << std::endl;

    /// Non-Cartesian
    typedef bg::model::point<double, 2, bg::cs::spherical_equatorial<bg::degree>> spherical_point;
    // spherical_point amsterdam(4.90, 52.37);
    // spherical_point paris(2.35, 48.86);
    spherical_point amsterdam(23.725750 , 37.971536);
    spherical_point paris(4.3826169, 50.8119483);
    double const earth_radius = 3959; // miles
    std::cout << "Distance between Amsterdam and Paris in miles: " << bg::distance(amsterdam, paris) * earth_radius /** 1.60934*/ << std::endl;

    // calculation for below code is wrong - TODO
    // using geographic_point = bg::model::point<double, 2, bg::cs::geographic<bg::degree>>;
    // std::cout << "Distance between Amsterdam and Paris in Km: " << 
    //     bg::distance(geographic_point(23.725750 , 37.971536), geographic_point(4.3826169, 50.8119483)) << std::endl;
    // std::cout << std::endl;

    /// Check if a 2D polygon is convex or concave
    std::cout << "Is the polygon convex: " << std::boolalpha << IsConvex(poly) << std::endl;
    std::cout << std::endl;

    /// Sort 3D points
    std::vector<Point3D> points3d{
        Point3D(0, 0, 0), Point3D(0, 0, 1), Point3D(0, 1, 0), Point3D(0, 1, 1), Point3D(1, 0, 0), Point3D(1, 0, 1), Point3D(1, 1, 0), Point3D(1, 1, 1)};
    std::random_shuffle(points3d.begin(), points3d.end());
    std::sort(points3d.begin(), points3d.end(), [](const Point3D &p1, const Point3D &p2) {
        using boost::geometry::get;
        if (get<0>(p1) < get<0>(p2)) return true;
        if (get<0>(p1) > get<0>(p2)) return false;
        if (get<1>(p1) < get<1>(p2)) return true;
        if (get<1>(p1) > get<1>(p2)) return false;
        return get<2>(p1) < get<2>(p2); 
    });
    std::cout << "Sorted by X-Y-Z" << std::endl;
    for (const auto &p : points3d)
        std::cout << p << std::endl;
    std::sort(points3d.begin(), points3d.end(), OrderByZ());
    std::cout << "Sorted by Z-Y-X" << std::endl;
    for (const auto &p : points3d)
        std::cout << p << std::endl;
    std::cout << std::endl;

    /// Boost.Geometry.Index - R-tree
    // spatial indexes to accelerate searching for objects in space
    // Each R-tree's node store a box describing the space occupied by its children nodes. At the bottom of the structure,
    // there are leaf-nodes which contains values (geometric objects representations)
    typedef bg::model::point<float, 2, bg::cs::cartesian> point;
    typedef bg::model::box<point> box;
    typedef std::pair<box, unsigned> value; // typically, store pair<Box, MyGeometryId> in the R-tree
    // create the rtree using default constructor
    bgi::rtree<value, bgi::quadratic<16>> rtree;
    for (auto i = 0; i < 10; ++i)
    {
        box b(point(i + 0.0f, i + 0.0f), point(i + 0.5f, i + 0.5f));
        rtree.insert(std::make_pair(b, i));
    }
    // spatial queries: find values intersecting some area defined by a box
    box query_box(point(0, 0), point(5, 5));
    std::vector<value> result_s;
    rtree.query(bgi::intersects(query_box), std::back_inserter(result_s));
    // k-nearest neighbor search
    std::vector<value> result_n;
    rtree.query(bgi::nearest(point(0, 0), 5), std::back_inserter(result_n));

    std::cout << "spatial query box:" << std::endl;
    std::cout << bg::wkt<box>(query_box) << std::endl;
    std::cout << "spatial query result:" << std::endl;
    // std::for_each(value const& v, result_s)
    BOOST_FOREACH (const value &v, result_s)
    {
        std::cout << bg::wkt<box>(v.first) << " - " << v.second << std::endl;
    }

    std::cout << "knn query point:" << std::endl;
    std::cout << bg::wkt<point>(point(0, 0)) << std::endl;
    std::cout << "knn query result:" << std::endl;
    BOOST_FOREACH (const value &v, result_n)
        std::cout << bg::wkt<box>(v.first) << " - " << v.second << std::endl;

    return 0;
}