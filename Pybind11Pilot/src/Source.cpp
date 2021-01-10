#include <pybind11/pybind11.h>
#include <pybind11/stl.h>   // use stl containers

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i = 1, int j = 2) {
    return i + j;
}

class Point final
{
public:
    Point() = default;
    Point(int a) : p_{a} {};

    int get() const { return p_; };
    void set(int a) { p_ = a; };

private:
    int p_ = 0;
};

namespace py = pybind11;

PYBIND11_MODULE(Pybind11Pilot, m) {
    m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: Pybind11Pilot

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";

    m.def("info", []() {
        py::print("C++ version = ", __cplusplus);
        // py::print("g++ version = ", __VERSION__);  // not working
        // py::print("libstdc++ = ", __GLIBCXX__);
    });

    // with keyword arguments, default arguments
    m.def("add", &add, "A function which adds two numbers",
          py::arg("i") = 1, py::arg("j") = 2);

    m.def("subtract", [](int i, int j) { return i - j; }, R"pbdoc(
        Subtract two numbers

        Some other explanation about the subtract function.
    )pbdoc");

    m.def("multiply", [=](int a, int b) {return a*b;}, R"pbdoc(
        multiply two numbers

        Some other explanation about the subtract function.
    )pbdoc");

    // exposing variables
    m.attr("the_answer") = 42;
    py::object world = py::cast("World");
    m.attr("what") = world;

    // use STL containers: string, tuple, vector
    m.def("use_str", [](const std::string& str) {
        py::print(str);
        return str+"!!";
    });

    m.def("use_tuple", [](std::tuple<int, int, std::string> x) {
        std::get<0>(x)++;
        std::get<1>(x)++;
        std::get<2>(x)+="??";
        return x;
    });

    m.def("use_list", [](const std::vector<int>& v) {
        auto vv = v;
        py::print("input : ", vv);
        vv.push_back(100);
        return vv;
    });

    // class
    py::class_<Point>(m, "Point")
        .def(py::init())
        .def(py::init<int>())
        .def("get", &Point::get)
        .def("set", &Point::set)
        .def("__repr__", [](const Point& p) {
            return "<Point with value: " + std::to_string(p.get()) + " >";
        });

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
