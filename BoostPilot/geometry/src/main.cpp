#include <boost/geometry.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/ring.hpp>
#include <iostream>
BOOST_GEOMETRY_REGISTER_C_ARRAY_CS(boost::geometry::cs::cartesian)
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(boost::geometry::cs::cartesian)

#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <functional>
#include <vector>

// Boost.Geometry is header only. There is not linking with any library
// necessary

namespace bg  = boost::geometry;
namespace bgi = boost::geometry::index;

bool IsConvex(/*const*/ boost::geometry::model::polygon<
              boost::geometry::model::d2::point_xy<double>>& polygon) {
    boost::geometry::correct(polygon);
    boost::geometry::model::polygon<
        boost::geometry::model::d2::point_xy<double>>
        hull;
    boost::geometry::convex_hull(polygon, hull);

    return boost::geometry::area(hull) == boost::geometry::area(polygon);
}

using Point3D =
    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>;

std::ostream& operator<<(std::ostream& os, const Point3D& p) {
    using boost::geometry::get;
    os << "(" << get<0>(p) << ", " << get<1>(p) << ", " << get<2>(p) << ")";
    return os;
}

std::function<bool(const Point3D& p1, const Point3D& p2)> OrderByZ() noexcept {
    return [](const Point3D& p1, const Point3D& p2) {
        using boost::geometry::get;
        if (get<2>(p1) < get<2>(p2))
            return true;
        if (get<2>(p1) > get<2>(p2))
            return false;
        if (get<1>(p1) < get<1>(p2))
            return true;
        if (get<1>(p1) > get<1>(p2))
            return false;
        return get<0>(p1) < get<0>(p2);
    };
}

void cartesianPointTest() {
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
    auto   d    = bg::distance(a, b);
    std::cout << "Distance with C array a-b is: " << d << std::endl;
    std::cout << std::endl;

    /// Polygon from set of ponts
    // double points[][2] = {{2.0, 1.3}, {4.1, 3.0}, {5.3, 2.6}, {2.9, 0.7},
    // {2.0, 1.3}};
    std::vector<Coordinate2D> points{
        {2.0, 1.3}, {4.1, 3.0}, {5.3, 2.6}, {2.9, 0.7}, {2.0, 1.3}};
    bg::model::polygon<Coordinate2D> poly;
    boost::geometry::append(poly, points);
    boost::tuple<double, double> tp = boost::make_tuple(3.7, 2.0);
    std::cout << "Point p is in polygon? " << std::boolalpha
              << bg::within(tp, poly) << std::endl;
    std::cout << "Polygon area: " << bg::area(poly) << std::endl;
    // mix C array point with Boost.Tuple point
    std::cout << "Distance of mix point a-p is: " << bg::distance(a, tp)
              << std::endl;
    bg::model::ring<Coordinate2D> points_again;
    bg::convert(poly, points_again);
    // assert(poitns == points_again);
    assert(std::equal(points.begin(), points.end(), points_again.begin(),
                      [](const Coordinate2D& a, const Coordinate2D& b) {
                          return bg::equals(a, b);
                      }));
    std::cout << std::endl;

    /// Check if a 2D polygon is convex or concave
    std::cout << "Is the polygon convex: " << std::boolalpha << IsConvex(poly)
              << std::endl;
    std::cout << std::endl;

    /// check distance between a point and polygon
    std::vector<Coordinate2D> poly_points = {
        {0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}};
    bg::model::polygon<Coordinate2D> polyg;
    bg::append(polyg, poly_points);
    Coordinate2D ego      = {-1, -1};
    auto         distance = std::numeric_limits<double>::max();
    if (boost::geometry::within(ego, polyg)) {
        boost::geometry::for_each_segment(
            polyg, [&distance, &ego](const auto& segment) {
                distance = std::min<float>(
                    distance, boost::geometry::distance(segment, ego));
            });
    } else {
        distance = bg::distance(ego, polyg);
    }

    std::cout << std::boolalpha << bg::within(ego, polyg) << ", " << distance
              << std::endl;
}

void point3DSortTest() {
    /// Sort 3D points
    std::vector<Point3D> points3d{
        Point3D(0, 0, 0), Point3D(0, 0, 1), Point3D(0, 1, 0), Point3D(0, 1, 1),
        Point3D(1, 0, 0), Point3D(1, 0, 1), Point3D(1, 1, 0), Point3D(1, 1, 1)};
    std::random_shuffle(points3d.begin(), points3d.end());
    std::sort(points3d.begin(), points3d.end(),
              [](const Point3D& p1, const Point3D& p2) {
                  using boost::geometry::get;
                  if (get<0>(p1) < get<0>(p2))
                      return true;
                  if (get<0>(p1) > get<0>(p2))
                      return false;
                  if (get<1>(p1) < get<1>(p2))
                      return true;
                  if (get<1>(p1) > get<1>(p2))
                      return false;
                  return get<2>(p1) < get<2>(p2);
              });
    std::cout << "Sorted by X-Y-Z" << std::endl;
    for (const auto& p : points3d)
        std::cout << p << std::endl;
    std::sort(points3d.begin(), points3d.end(), OrderByZ());
    std::cout << "Sorted by Z-Y-X" << std::endl;
    for (const auto& p : points3d)
        std::cout << p << std::endl;
    std::cout << std::endl;
}

void nonCartesianPointTest() {
    /// Non-Cartesian
    typedef bg::model::point<double, 2,
                             bg::cs::spherical_equatorial<bg::degree>>
        spherical_point;
    // spherical_point amsterdam(4.90, 52.37);
    // spherical_point paris(2.35, 48.86);
    spherical_point amsterdam(23.725750, 37.971536);
    spherical_point paris(4.3826169, 50.8119483);
    double const    earth_radius = 3959; // miles
    std::cout
        << "Distance between Amsterdam and Paris in km - spherical_equatrial: "
        << bg::distance(amsterdam, paris) * earth_radius * 1.60934 << std::endl;

    // calculation for below code is wrong - TODO
    using geographic_point =
        bg::model::point<double, 2, bg::cs::geographic<bg::degree>>;
    geographic_point amsterdam_pt = {23.725750, 37.971536};
    geographic_point paris_pt{4.3826169, 50.8119483};
    std::cout << "Distance between Amsterdam and Paris in Km - geographic: "
              << bg::distance(amsterdam_pt, paris_pt) / 1000 << std::endl;
    // std::cout << std::endl;

    typedef bg::srs::spheroid<double>               stype;
    typedef bg::strategy::distance::vincenty<stype> vincenty_type;
    // spherical_point point1(-73.787500, 2.179167),
    //             point2(106.139064, -2.162200);
    spherical_point point1(-74.0, 40.0),
        point2(-73.99994144771219, 40.00007799595619);
    double relative_distance = bg::distance(point1, point2, vincenty_type());
    std::cout.precision(15);
    std::cout << "relative distance of nearly antipodal points: "
              << relative_distance << std::endl;
    // corresponding direct problem for recovering
    bg::formula::result_direct<double> result;
    bg::srs::spheroid<double>          spheroid(6378137.0, 6356752.3142451793);
    typedef bg::formula::vincenty_direct<double, true, false, false, false>
                 vincenty_direct_type;
    const double pi = 3.1415926535;
    result = vincenty_direct_type::apply(-74.0 / 180 * pi, 40.0 / 180 * pi, 10,
                                         30 / 180.0 * 3.1415926535, spheroid);
    std::cout << "vincenty_direct: " << result.lat2 / pi * 180.0 << ", "
              << result.lon2 / pi * 180.0 << std::endl;

    // andoyer method
    typedef bg::strategy::andoyer::direct<double, true, false, false, false>
         direct_t;
    auto dir_r = direct_t::apply(-74.0 / 180 * pi, 40.0 / 180 * pi, 10,
                                 30 / 180.0 * 3.1415926535, spheroid);
    std::cout << "andoyer direct: " << dir_r.lat2 / pi * 180.0 << ", "
              << dir_r.lon2 / pi * 180.0 << std::endl;
}

void rtreeTest() {
    /// Boost.Geometry.Index - R-tree
    // spatial indexes to accelerate searching for objects in space
    // Each R-tree's node store a box describing the space occupied by its
    // children nodes. At the bottom of the structure, there are leaf-nodes
    // which contains values (geometric objects representations)
    typedef bg::model::point<float, 2, bg::cs::cartesian> point;
    typedef bg::model::box<point>                         box;
    typedef std::pair<box, unsigned>
        value; // typically, store pair<Box, MyGeometryId> in the R-tree
    // create the rtree using default constructor
    bgi::rtree<value, bgi::quadratic<16>> rtree;
    for (auto i = 0; i < 10; ++i) {
        box b(point(i + 0.0f, i + 0.0f), point(i + 0.5f, i + 0.5f));
        rtree.insert(std::make_pair(b, i));
    }
    // spatial queries: find values intersecting some area defined by a box
    box                query_box(point(0, 0), point(5, 5));
    std::vector<value> result_s;
    rtree.query(bgi::intersects(query_box), std::back_inserter(result_s));
    // k-nearest neighbor search
    std::vector<value> result_n;
    rtree.query(bgi::nearest(point(0, 0), 3), std::back_inserter(result_n));
    std::cout << "intersection size: " << result_s.size() << ", "
              << "k-nearest size: " << result_n.size() << std::endl;

    std::cout << "spatial query box:" << std::endl;
    std::cout << bg::wkt<box>(query_box) << std::endl;
    std::cout << "spatial query result:" << std::endl;
    // std::for_each(value const& v, result_s)
    BOOST_FOREACH (const value& v, result_s) {
        std::cout << bg::wkt<box>(v.first) << " - " << v.second << std::endl;
    }

    std::cout << "knn query point:" << std::endl;
    std::cout << bg::wkt<point>(point(0, 0)) << std::endl;
    std::cout << "knn query result:" << std::endl;
    BOOST_FOREACH (const value& v, result_n)
        std::cout << bg::wkt<box>(v.first) << " - " << v.second << std::endl;
}

void polygonTest() {
    using point   = bg::model::point<double, 2, bg::cs::cartesian>;
    using polygon = bg::model::polygon<point>;
    using box     = bg::model::box<point>;

    // using value = std::pair<box, std::size_t>;
    // using value = std::pair<polygon, std::size_t>;
    using value = std::pair<box, std::pair<std::size_t, bool>>;
    using rtree = bgi::rtree<value, bgi::rstar<16>>;

    std::vector<point> points{
        {2.0, 1.3}, {2.4, 1.7}, {2.8, 1.8}, {3.4, 1.2}, {3.7, 1.6},
        {3.4, 2.0}, {4.1, 3.0}, {5.3, 2.6}, {5.4, 1.2}, {4.9, 0.8},
        {2.9, 0.7}, {2.0, 1.3}}; // closing point is opening point};
    polygon poly;
    // can form polygon from single point???
    polygon poly_single;
    bg::assign_points(poly_single, points[0]);
    std::cout << bg::dsv(poly_single) << std::endl
              << "outter rings: " << poly_single.outer().size() << std::endl;
    // boost::geometry::append(poly, points);
    bg::assign_points(poly, points);

    bg::correct(poly);
    // poly.clear();
    std::cout << "Polygon " << bg::dsv(poly) << " has an area of "
              << bg::area(poly) << std::endl;
    std::cout << "number of points in outer ring: " << poly.outer().size()
              << std::endl;
    std::cout << "number of points in innder ring: " << poly.inners().size()
              << std::endl;

    box b1;
    bg::envelope(poly, b1);
    std::cout << "Envelope " << bg::dsv(b1) << std::endl;
    point center;
    bg::centroid(poly, center);
    std::cout << "CoG of poly: " << bg::dsv(center) << std::endl;
    // Polygons can have one or more inner rings, also called holes, islands,
    // interior rings.
    {
        poly.inners().resize(1);
        bg::model::ring<point>& inner     = poly.inners().back();
        const double            coor[][2] = {
                       {4.0, 2.0}, {4.2, 1.4}, {4.8, 1.9}, {4.4, 2.2}, {4.0, 2.0}};
        bg::assign_points(inner, coor);
    }
    bg::correct(poly);
    std::cout << "with inner rings: " << bg::dsv(poly) << std::endl;
    std::cout << "new area: " << bg::area(poly) << std::endl;
    bg::centroid(poly, center);
    std::cout << "new CoG: " << bg::dsv(center) << std::endl;
    std::cout << "number of points in innder ring: "
              << /*poly.inners().size()*/ poly.inners().at(0).size()
              << std::endl;
    // point within
    std::cout << "point within polygon: " << std::boolalpha
              << bg::within(point{3.0, 2.0}, poly) << ", " << std::boolalpha
              << bg::within(point{3.7, 2.0}, poly) << std::endl;
    // clip polygon using a box
    box cb(point{1.5, 1.5}, bg::make<point>(4.5, 2.5));
    using polygon_list = std::vector<polygon>;
    polygon_list lst;
    bg::intersection(cb, poly, lst);
    std::cout << "clipped output polygons" << std::endl;
    for (auto it = lst.cbegin(); it != lst.cend(); ++it) {
        std::cout << bg::dsv(*it) << std::endl;
    }

    // test with rtree
    std::vector<std::vector<point>> polygon_pts = {
        {{0, 0}, {0, 1}, {1, 0} /*, {0, 0}*/},
        {{1, 1}, {1, 2}, {2, 1}, {1, 1}},
        {{2, 2}, {2, 3}, {3, 2}, {2, 2}},
        {{3, 3}, {3, 4}, {4, 3}, {3, 3}}};
    std::vector<polygon> polys(polygon_pts.size());
    rtree                rt;
    for (size_t i = 0; i < polygon_pts.size(); ++i) {
        bg::assign_points(polys[i], polygon_pts[i]);
        box b = bg::return_envelope<box>(
            polys[i]); // create one envelope box for each polygon
        rt.insert(std::make_pair(b, std::make_pair(i, true)));
        // rt.insert(std::make_pair(polys[i], i));
    }

    polygon qpoly;
    bg::read_wkt("POLYGON((0.25 0.25,0.5 1.5,0.9 0.9,1.5 0.5,0.25 0.25))",
                 qpoly);
    box qbox = bg::return_buffer<box>(bg::return_envelope<box>(qpoly), 0.0001);
    std::vector<value> result;
    // we can't query polygon directly?
    std::cout
        << "rtree query for polygon (corresponding box) insersection query: "
        << std::endl;
    rt.query(bgi::intersects(qbox), std::back_inserter(result));
    for (value const& v : result) {
        std::cout << bg::wkt(polys[v.second.first])
                  << (bg::intersects(polys[v.second.first], qpoly)
                          ? " intersects"
                          : " not intersects")
                  << std::endl;
    }

    // k-nearest neighbor search
    point              pt{0.9, 0.9};
    std::vector<value> result_n;
    rt.query(bgi::nearest(pt, 1), std::back_inserter(result_n));
    std::cout << "k-nearest point search of point to polygon (corresponding "
                 "box) size: "
              << result_n.size() << std::endl;
    for (const value& v : result_n) {
        std::cout << bg::wkt(polys[v.second.first]) << std::endl;
    }

    // distance to all polygons
    std::vector<double> distances(polys.size());
    auto distanceToPolygon = [](const point& ego, const polygon& polyg) {
        auto distance = std::numeric_limits<double>::max();
        if (boost::geometry::within(ego, polyg)) {
            boost::geometry::for_each_segment(
                polyg, [&distance, &ego](const auto& segment) {
                    distance = std::min<float>(
                        distance, boost::geometry::distance(segment, ego));
                });
        } else {
            distance = bg::distance(ego, polyg);
        }
        return distance;
    };
    for (int i = 0; i < polys.size(); ++i) {
        distances[i] = distanceToPolygon(pt, polys[i]);
    }
    for (const auto& d : distances)
        std::cout << d << ", ";
    std::cout << std::endl;
}

int main() {
    // cartesianPointTest();
    // point3DSortTest();
    nonCartesianPointTest();
    // rtreeTest();
    // polygonTest();

    return 0;
}