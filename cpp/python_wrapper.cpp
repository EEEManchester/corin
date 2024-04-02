#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/CorinHexapod.h>
#include <dqrobotics/robot_modeling/DQ_LeggedRobot.h>

namespace py = pybind11;
using namespace std;
using namespace DQ_robotics;
constexpr auto byref = py::return_value_policy::reference_internal;

PYBIND11_MODULE(lib_Legged_Robotics_CPP_BINDINGS, m) {
    m.doc() = "Python wrapper around C++ implementations of the Corin object in the legged robotics fork";

    // Bind corin object
    py::class_<CorinHexapod>(m, "DQ_CorinHexapod")
        .def(py::init<>())
        .def("kinematics", &CorinHexapod::kinematics);

    // Bind the legged robot object
    py::class_<DQ_LeggedRobot>(m, "DQ_LeggedRobot")
        .def(py::init<std::string>())
        .def("foot_position_as_numpy", &DQ_LeggedRobot::foot_position_as_vector);
}