#include <pybind11/pybind11.h>

int add(int i = 1, int j = 2) {
    return i+j;
}

namespace py = pybind11;

PYBIND11_MODULE(example, m) {
    m.doc() = "pybind11 example plugin";
    // m.def("add", &add, "A function which adds two numbers");
    // with keyword arguments, default arguments
    m.def("add", &add, "A function which adds two numbers",
          py::arg("i") = 1, py::arg("j") = 2);
    m.def("subtract", [](int i, int j) {return i-j;},"A function which substracts numbers");

    // exposing variables
    m.attr("the_answer") = 42;
    py::object world = py::cast("World");
    m.attr("what") = world;
#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
