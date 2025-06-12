/**
(C) Copyright 2025 Murilo Marinho (murilomarinho@ieee.org)
*/

#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <M3_SerialManipulatorSimulatorFriendly.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
using namespace DQ_robotics;

PYBIND11_MODULE(_core, m) {

    py::class_<
        M3_SerialManipulatorSimulatorFriendly,
        std::shared_ptr<M3_SerialManipulatorSimulatorFriendly>,
        DQ_SerialManipulator
        > dqserialmanipulatorsimulatorfriendly_py(m, "M3_SerialManipulatorSimulatorFriendly");
    dqserialmanipulatorsimulatorfriendly_py.def(py::init<std::vector<DQ>,std::vector<DQ>,std::vector<M3_SerialManipulatorSimulatorFriendly::ActuationType>>());

    ///Methods
    //Overrides from DQ_SerialManipulator
    dqserialmanipulatorsimulatorfriendly_py.def("raw_pose_jacobian",  (MatrixXd (M3_SerialManipulatorSimulatorFriendly::*)(const VectorXd&, const int&) const)&M3_SerialManipulatorSimulatorFriendly::raw_pose_jacobian, "Retrieves the raw pose Jacobian.");
    dqserialmanipulatorsimulatorfriendly_py.def("raw_fkm",            (DQ (M3_SerialManipulatorSimulatorFriendly::*)(const VectorXd&, const int&) const)&M3_SerialManipulatorSimulatorFriendly::raw_fkm,                 "Retrieves the raw FKM.");
    dqserialmanipulatorsimulatorfriendly_py.def("raw_pose_jacobian_derivative",(MatrixXd (M3_SerialManipulatorSimulatorFriendly::*)(const VectorXd&, const VectorXd&, const int&) const)
                                                                                    &M3_SerialManipulatorSimulatorFriendly::raw_pose_jacobian_derivative, "Retrieves the raw pose Jacobian derivative.");

    py::enum_<M3_SerialManipulatorSimulatorFriendly::ActuationType>(dqserialmanipulatorsimulatorfriendly_py, "ActuationType")
        .value("RZ", M3_SerialManipulatorSimulatorFriendly::ActuationType::RZ)
        .value("RY", M3_SerialManipulatorSimulatorFriendly::ActuationType::RY)
        .value("RX", M3_SerialManipulatorSimulatorFriendly::ActuationType::RX)
        .value("TZ", M3_SerialManipulatorSimulatorFriendly::ActuationType::TZ)
        .value("TY", M3_SerialManipulatorSimulatorFriendly::ActuationType::TY)
        .value("TX", M3_SerialManipulatorSimulatorFriendly::ActuationType::TX)
        .export_values();


#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
