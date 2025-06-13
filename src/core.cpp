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

    m.doc() = R"coredoc(
        marinholab.working.needlemanipulation
        -------------------------------------

        .. currentmodule:: needlemanipulation

        .. autosummary::
           :toctree: _generate

           M3_SerialManipulatorSimulatorFriendly
    )coredoc";

    py::class_<
        M3_SerialManipulatorSimulatorFriendly,
        std::shared_ptr<M3_SerialManipulatorSimulatorFriendly>,
        DQ_SerialManipulator
        > dqserialmanipulatorsimulatorfriendly_py(m, "M3_SerialManipulatorSimulatorFriendly");

    //M3_SerialManipulatorSimulatorFriendly(const std::vector<DQ>& offset_before,
    //                                      const std::vector<DQ>& offset_after,
    //                                      const std::vector<ActuationType>& actuation_types);
    dqserialmanipulatorsimulatorfriendly_py.def(py::init<std::vector<DQ>,std::vector<DQ>,std::vector<M3_SerialManipulatorSimulatorFriendly::ActuationType>>(),
    py::arg("offset_before"),py::arg("offset_after"),py::arg("actuation_types"),
    R"coredoc(
        The M3_SerialManipulatorSimulatorFriendly constructor.

        :param offset_before: A list of DQ representing the offset of each joint transformation before actuation.
        :type offset_before: List[DQ]
        :param offset_after: A list of DQ representing the offset of each joint transformation after actuation.
        :type offset_after: List[DQ]
        :param actuation_types: A list of M3_SerialManipulatorSimulatorFriendly.ActuationType denoting the actuation
                                type and axis of each joint.
        :type f: List[M3_SerialManipulatorSimulatorFriendly.ActuationType]
        :rtype: M3_SerialManipulatorSimulatorFriendly
    )coredoc");
    ///Methods
    //Overrides from DQ_SerialManipulator
    dqserialmanipulatorsimulatorfriendly_py.def("raw_pose_jacobian",  (MatrixXd (M3_SerialManipulatorSimulatorFriendly::*)(const VectorXd&, const int&) const)&M3_SerialManipulatorSimulatorFriendly::raw_pose_jacobian, "Retrieves the raw pose Jacobian.");
    dqserialmanipulatorsimulatorfriendly_py.def("raw_fkm",            (DQ (M3_SerialManipulatorSimulatorFriendly::*)(const VectorXd&, const int&) const)&M3_SerialManipulatorSimulatorFriendly::raw_fkm,                 "Retrieves the raw FKM.");
    dqserialmanipulatorsimulatorfriendly_py.def("raw_pose_jacobian_derivative",(MatrixXd (M3_SerialManipulatorSimulatorFriendly::*)(const VectorXd&, const VectorXd&, const int&) const)
                                                                                    &M3_SerialManipulatorSimulatorFriendly::raw_pose_jacobian_derivative, "Retrieves the raw pose Jacobian derivative.");

    py::enum_<M3_SerialManipulatorSimulatorFriendly::ActuationType>(dqserialmanipulatorsimulatorfriendly_py, "ActuationType")
        .value("RZ", M3_SerialManipulatorSimulatorFriendly::ActuationType::RZ, "Revolution about the z-axis")
        .value("RY", M3_SerialManipulatorSimulatorFriendly::ActuationType::RY, "Revolution about the y-axis")
        .value("RX", M3_SerialManipulatorSimulatorFriendly::ActuationType::RX, "Revolution about the x-axis")
        .value("TZ", M3_SerialManipulatorSimulatorFriendly::ActuationType::TZ,  "Translation along the z-axis")
        .value("TY", M3_SerialManipulatorSimulatorFriendly::ActuationType::TY,  "Translation along the y-axis")
        .value("TX", M3_SerialManipulatorSimulatorFriendly::ActuationType::TX,  "Translation along the x-axis")
        .export_values();


#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
