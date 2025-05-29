/**
(C) Copyright 2019-2025 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
1.  Murilo M. Marinho        (murilomarinho@ieee.org)
        - Initial implementation.

2.  Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
        - Added bindings for the following methods
          set_stepping_mode(), get_object_handle(), get_object_handle()
          get_joint_{velocities, torques}(), set_joint_target_velocities(),set_joint_torques()
*/

#include "../../module.h"

//Default arguments added with:
//https://pybind11.readthedocs.io/en/stable/basics.html#default-args

void init_DQ_CoppeliaSimInterfaceZMQ_py(py::module& m)
{
    /*****************************************************
     *  VrepInterface
     * **************************************************/
    py::class_<
            DQ_CoppeliaSimInterfaceZMQ,
            std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ>,
            DQ_CoppeliaSimInterface
            > c(m,"DQ_CoppeliaSimInterfaceZMQ");
    c.def(py::init<>());

    c.def("connect",(bool (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&, const int&, const int&))&DQ_CoppeliaSimInterfaceZMQ::connect,
                            py::arg("host") = "localhost", py::arg("port") = 23000, py::arg("TIMEOUT_IN_MILISECONDS") = 2000,
                            "establishes a connection between the client (your code) and the host (the computer running the CoppeliaSim scene.");
    c.def("connect",(bool (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&, const int&, const int&, const int&))&DQ_CoppeliaSimInterfaceZMQ::connect,"Connects to CoppeliaSim with a given ip.");

    c.def("disconnect",    &DQ_CoppeliaSimInterfaceZMQ::disconnect,"Disconnects from CoppeliaSim.");
    c.def("disconnect_all",&DQ_CoppeliaSimInterfaceZMQ::disconnect_all,"Disconnect all from CoppeliaSim");

    c.def("start_simulation",&DQ_CoppeliaSimInterfaceZMQ::start_simulation,"Start simulation");
    c.def("stop_simulation", &DQ_CoppeliaSimInterfaceZMQ::stop_simulation,"Stops simulation");

    c.def("set_stepping_mode", (void (DQ_CoppeliaSimInterfaceZMQ::*) (const bool&))&DQ_CoppeliaSimInterfaceZMQ::set_stepping_mode, "enables or disables the stepping mode (formerly known as synchronous mode).");
    c.def("set_synchronous", (void (DQ_CoppeliaSimInterfaceZMQ::*) (const bool&))&DQ_CoppeliaSimInterfaceZMQ::set_synchronous, "Sets synchronous mode");

    c.def("trigger_next_simulation_step", &DQ_CoppeliaSimInterfaceZMQ::trigger_next_simulation_step, "Sends a synchronization trigger signal to the server.");

    c.def("wait_for_simulation_step_to_end", &DQ_CoppeliaSimInterfaceZMQ::wait_for_simulation_step_to_end, "Waits until the simulation step is finished.");

    c.def("get_object_translation",
                           (DQ (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&))&DQ_CoppeliaSimInterfaceZMQ::get_object_translation,
                           "Gets object translation.");

    c.def("set_object_translation",
                           (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&, const DQ&))&DQ_CoppeliaSimInterfaceZMQ::set_object_translation,
                           "Sets object translation.");

    c.def("get_object_rotation",
                           (DQ (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&))&DQ_CoppeliaSimInterfaceZMQ::get_object_rotation,
                           "Gets object rotation.");

    c.def("set_object_rotation",
                           (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&, const DQ&))&DQ_CoppeliaSimInterfaceZMQ::set_object_rotation,
                           "Sets object rotation.");

    c.def("get_object_pose",
                           (DQ (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&))&DQ_CoppeliaSimInterfaceZMQ::get_object_pose,
                           "Gets object pose.");

    c.def("set_object_pose",
                           (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&, const DQ&))&DQ_CoppeliaSimInterfaceZMQ::set_object_pose,
                           "Sets object pose.");

    c.def("get_object_handle",
                            (int (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&))&::DQ_CoppeliaSimInterfaceZMQ::get_object_handle,
                            "gets the object handle from CoppeliaSim.");

    c.def("get_object_handles",
                            (VectorXd (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&))&DQ_CoppeliaSimInterfaceZMQ::get_object_handles,
                            "returns a vector containing the object handles.");

    c.def("set_joint_positions",
                           (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&, const VectorXd&))&DQ_CoppeliaSimInterfaceZMQ::set_joint_positions,
                           "Set joint positions");

    c.def("set_joint_target_positions",
                           (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&, const VectorXd&))&DQ_CoppeliaSimInterfaceZMQ::set_joint_target_positions,
                           "Set joint positions");

    c.def("get_joint_positions",
                           (VectorXd (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&))&DQ_CoppeliaSimInterfaceZMQ::get_joint_positions,
                           "Get joint positions");

    c.def("get_joint_velocities",
                            (VectorXd (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&))&DQ_CoppeliaSimInterfaceZMQ::get_joint_velocities,
                            "gets the joint velocities in the CoppeliaSim scene.");

    c.def("set_joint_target_velocities",
                            (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&, const VectorXd&))&DQ_CoppeliaSimInterfaceZMQ::set_joint_target_velocities,
                            "sets the joint target velocities in the CoppeliaSim scene. "
                            "This method requires a dynamics enabled scene, and joints in dynamic mode with velocity control mode.");

    c.def("get_joint_torques",
                            (VectorXd (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&))&DQ_CoppeliaSimInterfaceZMQ::get_joint_torques,
                            "gets the joint torques in the CoppeliaSim scene.");

    c.def("set_joint_torques",
                            (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&, const VectorXd&))&DQ_CoppeliaSimInterfaceZMQ::set_joint_torques,
                            "sets the joint torques in the CoppeliaSim scene. "
                            "This method requires a dynamics enabled scene, and joints in dynamic mode with force control mode.");


}
