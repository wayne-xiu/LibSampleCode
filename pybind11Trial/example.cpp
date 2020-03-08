#include <pybind11/pybind11.h>

int add(int i, int j) {
    return i+j;
}

namespace py = pybind11;

PYBIND11_MODULE(example, m) {
    m.doc() = "pybind11 example plugin";
    m.def("add", &add, "A function which adds two numbers");
    m.def("subtract", [](int i, int j) {return i-j;},"A function which substracts numbers");

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
