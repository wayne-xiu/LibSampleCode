#include <pybind11/pybind11.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i = 1, int j = 2) {
    return i + j;
}

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

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
