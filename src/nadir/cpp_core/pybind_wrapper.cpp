#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "ekf.h"

namespace py = pybind11;

PYBIND11_MODULE(nadir_ekf, m) {
    m.doc() = "Nadir Extended Kalman Filter C++ core via pybind11";

    py::class_<nadir::cpp_core::ExtendedKalmanFilter>(m, "ExtendedKalmanFilter")
        .def(py::init<double, double, double>(),
             py::arg("dt"), py::arg("process_noise") = 1.0, py::arg("measurement_noise") = 1.0)
        .def("predict", &nadir::cpp_core::ExtendedKalmanFilter::predict,
             "Predict the next state")
        .def("update", &nadir::cpp_core::ExtendedKalmanFilter::update,
             py::arg("measurement"),
             "Update the state with a new measurement [x, y, z]")
        .def("get_state", &nadir::cpp_core::ExtendedKalmanFilter::get_state,
             "Get the current full state vector [x, y, z, vx, vy, vz]")
        .def("get_position", &nadir::cpp_core::ExtendedKalmanFilter::get_position,
             "Get the current position [x, y, z]")
        .def("get_velocity", &nadir::cpp_core::ExtendedKalmanFilter::get_velocity,
             "Get the current velocity [vx, vy, vz]")
        .def("reset", &nadir::cpp_core::ExtendedKalmanFilter::reset,
             py::arg("initial_state"),
             "Reset the filter with a new state");
}
