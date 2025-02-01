#include "QuadDDSComm.hpp"
// #include "PointCloudHandler.hpp"

QuadDDSComm::QuadDDSComm(const DATA_ACCESS_MODE& mode) : m_name("m2"), CommunicationManager("m2", mode) {
    initClass();
}
QuadDDSComm::QuadDDSComm(const std::string& name, const DATA_ACCESS_MODE& mode) : m_name(name), CommunicationManager(name, mode) {
    initClass();
}
QuadDDSComm::~QuadDDSComm() {
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void QuadDDSComm::initClass() {
    if (m_mode == DATA_ACCESS_MODE::PLANT) {
        // m_plant_time_sub_ptr = NULL;
        
        m_joint_cmd_sub_ptr.reset(new DDSSubscriber<JointData_>(
            "rt/" + m_name + "/joint_command",
            std::bind(&QuadDDSComm::get_joint_cmd_cb, this, std::placeholders::_1),
            DOMAIN_ID
        ));

        m_joint_command_msg = JointData_();
        m_sensor_data_pub_ptr.reset(new DDSPublisher<SensorData_>("rt/" + m_name + "/sensor_data", DOMAIN_ID));

        m_sensor_data_msg = SensorData_();

        // m_plant_time_pub_ptr.reset(new DDSPublisher<FloatScalar_>("rt/" + m_name + "/plant_time"));

        m_ground_truth_data_pub_ptr.reset(new DDSPublisher<QuadLog_>("rt/" + m_name + "/gt_data", DOMAIN_ID));
        m_ground_truth_data = QuadLog_();

    } else if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
        m_sensor_data_sub_ptr.reset(new DDSSubscriber<SensorData_>(
            "rt/" + m_name + "/sensor_data",
            std::bind(&QuadDDSComm::get_sensor_data_cb, this, std::placeholders::_1),
            DOMAIN_ID
        ));
        m_sensor_data_msg = SensorData_();

        m_joy_data_sub_ptr.reset(new DDSSubscriber<JoyData_>(
            "rt/joystick_data",
            std::bind(&QuadDDSComm::get_joystick_data_cb, this, std::placeholders::_1),
            DOMAIN_ID
        ));
        m_joy_data_msg = JoyData_();

        m_obstacle_coords_sub_ptr.reset(new DDSSubscriber<Point_>(
            "rt/" + m_name + "/obstacle_coords",
            std::bind(&QuadDDSComm::get_obstacle_coords_cb, this, std::placeholders::_1),
            DOMAIN_ID
        ));
        obstacle_coords = Point_();

        m_joint_cmd_pub_ptr.reset(new DDSPublisher<JointData_>(
            "rt/" + m_name + "/joint_command",
            DOMAIN_ID
        ));
        m_joint_command_msg = JointData_();

        // m_plant_time_sub_ptr.reset(new DDSSubscriber<FloatScalar_>(
        //     "/" + m_name + "/plant_time",
        //     std::bind(&QuadDDSComm::get_plant_time_cb, this, std::placeholders::_1)
        // ));

        m_estimated_data_pub_ptr.reset(new DDSPublisher<QuadLog_>("rt/" + m_name + "/estimated", DOMAIN_ID));
        m_estimated_data = QuadLog_();
        m_reference_data_pub_ptr.reset(new DDSPublisher<QuadLog_>("rt/" + m_name + "/reference", DOMAIN_ID));
        m_reference_data = QuadLog_();

        m_solver_stats_pub_ptr.reset(new DDSPublisher<SolverStats_>("rt/" + m_name + "/solver_stats", DOMAIN_ID));
        m_solver_stats = SolverStats_();

        m_base_err_pub_ptr.reset(new DDSPublisher<Point_>("rt/" + m_name + "/base_err", DOMAIN_ID));
        m_base_err = Point_();
    }
    
    setPlantTimePtr(&m_plant_time);

    m_startTimePoint = std::chrono::high_resolution_clock::now();
}

void QuadDDSComm::get_joint_cmd_cb(const JointData_& msg) {
    if (m_joint_command_data_ptr == NULL) {
        return;
    }
    for (int i = 0; i < 12; ++i) {
        m_joint_command_data_ptr->q(i) = msg.q()[i];
        m_joint_command_data_ptr->qd(i) = msg.dq()[i];
        m_joint_command_data_ptr->tau(i) = msg.tau()[i];
        m_joint_command_data_ptr->kp(i) = msg.kp()[i];
        m_joint_command_data_ptr->kd(i) = msg.kd()[i];
    }
}

void QuadDDSComm::get_sensor_data_cb(const SensorData_& msg) {

    if (m_sensor_data_ptr == NULL) {
        return;
    }

    m_communication_ready = true;

    std::unique_lock<std::mutex> sensor_lock(m_sensor_data_mutex);

    for (int i = 0; i < 12; ++i) {
        m_sensor_data_ptr->q(i) = msg.q()[i];
        m_sensor_data_ptr->qd(i) = msg.dq()[i];
        m_sensor_data_ptr->tau(i) = msg.tau_est()[i];
    }

    for (int i = 0; i < 4; ++i) {
        m_sensor_data_ptr->quat(i) = msg.quat()[i];
    }

    for (int i = 0; i < 3; ++i) {
        m_sensor_data_ptr->a_B(i) = msg.accel()[i];
        m_sensor_data_ptr->w_B(i) = msg.gyro()[i];
    }

    // std::cout << "Updated sensor data: " << m_sensor_data_ptr->q.transpose() << "\n";

    //debug code
    //std::cout << m_sensor_data_ptr->quat(0) << "," << m_sensor_data_ptr->quat(1) << "," << m_sensor_data_ptr->quat(2) << "," <<m_sensor_data_ptr->quat(3) << "\n";
}

void QuadDDSComm::get_joystick_data_cb(const JoyData_& msg) {
    if (m_joystick_data_ptr == NULL || !m_is_executor_ready) {
        std::cout << "Joystick pointer not set or executor is not ready!\n";
        return;
    }
    // m_joystick_data_ptr -> lx = -1 * msg.axes[0];
    // m_joystick_data_ptr -> ly = msg.axes[1];
    // m_joystick_data_ptr -> rx = -1 * msg.axes[2];
    // m_joystick_data_ptr -> ry = msg.axes[3];
    // m_joystick_data_ptr -> rt = msg.axes[4];
    // m_joystick_data_ptr -> lt = msg.axes[5];
    // m_joystick_data_ptr -> dx = msg.axes[6];
    // m_joystick_data_ptr -> dy = msg.axes[7];

    // // the mode is set by finding the index of the button that is pressed
    // m_joystick_data_ptr -> mode = std::distance(msg.buttons.begin(), std::find(msg.buttons.begin(), msg->buttons.end(), 1)) + 1;

    if (m_joystick_is_nintendo) {
        m_joystick_data_ptr->vel_x = -msg.axes()[1];
        m_joystick_data_ptr->vel_y = -msg.axes()[0];
        m_joystick_data_ptr->vel_yaw = -msg.axes()[2];
        m_joystick_data_ptr->pitch = -msg.axes()[3];
        if (msg.buttons()[0]) {
            m_joystick_data_ptr->mode = 1;
        } else if (msg.buttons()[1]) {
            m_joystick_data_ptr->mode = 4;
        } else if (msg.buttons()[3]) {
            m_joystick_data_ptr->mode = 3;
        } else if (msg.buttons()[2]) {
            m_joystick_data_ptr->mode = 2;
        }
        if (msg.axes()[5] > 0) {
            m_joystick_data_ptr -> up = msg.axes()[5];
        } else {
            m_joystick_data_ptr -> down = -msg.axes()[5];
        }
        if (msg.axes()[4] > 0) {
            m_joystick_data_ptr -> left = msg.axes()[4];
        } else {
            m_joystick_data_ptr -> right = -msg.axes()[4];
        }
    } else {
        // m_joystick_data_ptr->vel_x = -msg.axes()[1];
        // m_joystick_data_ptr->vel_y = -msg.axes()[0];
        // m_joystick_data_ptr->vel_yaw = -msg.axes()[3];
        m_joystick_data_ptr->roll = -msg.axes()[0];
        m_joystick_data_ptr->pitch = -msg.axes()[4];
        m_joystick_data_ptr->yaw = -msg.axes()[3];
        // if (-msg.buttons()[17] < 0){
        //         m_joystick_data_ptr->up = -msg.buttons()[17];
        // }
        // else {
        //       m_joystick_data_ptr->down = -msg.buttons()[17];
        // }



        if (msg.buttons()[0]) {
            m_joystick_data_ptr->mode = 1;
        } else if (msg.buttons()[1]) {
            m_joystick_data_ptr->mode = 2;
        } else if (msg.buttons()[3]) {
            m_joystick_data_ptr->mode = 3;
        } else if (msg.buttons()[2]) {
            m_joystick_data_ptr->mode = 4;
        }
        if (msg.axes()[5] > 0) {
            m_joystick_data_ptr -> up = msg.axes()[5];
        } else {
            m_joystick_data_ptr -> down = -msg.axes()[5];
        }
        if (msg.axes()[4] > 0) {
            m_joystick_data_ptr -> left = msg.axes()[4];
        } else {
            m_joystick_data_ptr -> right = -msg.axes()[4];
        }
    }
}

void QuadDDSComm::get_obstacle_coords_cb(const Point_& msg) {
    obstacle_coords = msg;
    // convertPointCloud2ToPCL(lidar_point_cloud, pcl_cloud);
    // // Get the nearest point to the origin
    // double obstacle_x = 0.0;
    // double obstacle_y = 0.0;
    // double obstacle_z = 0.0;
    // double distance = 0.0;
    // std::tie(obstacle_x, obstacle_y, obstacle_z, distance) = getDistanceFromNearestNeighbour(pcl_cloud, 1);

    vec3 obstacle_B = vec3::Zero();
    obstacle_B = pinocchio::Ry(M_PI) * vec3(obstacle_coords.x(), obstacle_coords.y(), obstacle_coords.z());

    m_sensor_data_ptr -> obstacle_coords = obstacle_B;

    // std::cout << "Nearest obstacle coords: " << m_estimation_data_ptr->obstacle_coords.transpose() << "\n";
}

// void QuadDDSComm::get_plant_time_cb(const FloatScalar_& time) {
//     m_plant_time = time.data();
// }

void QuadDDSComm::write_sensor_data() {

    if (m_sensor_data_ptr == NULL) {
        return;
    }

    std::unique_lock<std::mutex> sensor_lock(m_sensor_data_mutex);

    for (int i = 0; i < 12; ++i) {
        m_sensor_data_msg.q()[i] = m_sensor_data_ptr->q(i);
        m_sensor_data_msg.dq()[i] = m_sensor_data_ptr->qd(i);
        m_sensor_data_msg.tau_est()[i] = m_sensor_data_ptr->tau(i);
    }

    for (int i = 0; i < 3; ++i) {
        m_sensor_data_msg.accel()[i] = m_sensor_data_ptr->a_B(i);
        m_sensor_data_msg.gyro()[i] = m_sensor_data_ptr->w_B(i);
    }

    vec4 quat_calc = pinocchio::EulToQuat(m_sensor_data_ptr->eul);
    
    for (int i = 0; i < 4; ++i) {
        #ifdef USE_HARDWARE
            m_sensor_data_msg.quat()[i] = quat_calc(i);
        #else
            m_sensor_data_msg.quat()[i] = m_sensor_data_ptr->quat(i);
        #endif
    }
    
    m_sensor_data_pub_ptr -> publish(m_sensor_data_msg);
}

void QuadDDSComm::write_measurement_data() {
    if (m_estimation_data_ptr == NULL || m_measurement_data_ptr == NULL) {
        return;
    }
    // write estimated data and publish
    for (int i = 0; i < 4; ++i) {
        m_estimated_data.contact_state()[i] = m_estimation_data_ptr->cs(i);
        m_estimated_data.contact_prob()[i] = m_estimation_data_ptr->pc(i);
    }
    for (int i = 0; i < 12; ++i) {
        m_estimated_data.contact_force()[i] = m_measurement_data_ptr->estimated_contact_force(i);
    }
    m_estimated_data.base_position().x() = m_estimation_data_ptr->rB(0);
    m_estimated_data.base_position().y() = m_estimation_data_ptr->rB(1);
    m_estimated_data.base_position().z() = m_estimation_data_ptr->rB(2);

    m_estimated_data.base_orientation().x() = m_estimation_data_ptr->js(3);
    m_estimated_data.base_orientation().y() = m_estimation_data_ptr->js(4);
    m_estimated_data.base_orientation().z() = m_estimation_data_ptr->js(5);
    m_estimated_data.base_orientation().w() = m_estimation_data_ptr->js(6);

    m_estimated_data.linear_velocity().x() = m_estimation_data_ptr->jv(0);
    m_estimated_data.linear_velocity().y() = m_estimation_data_ptr->jv(1);
    m_estimated_data.linear_velocity().z() = m_estimation_data_ptr->jv(2);

    m_estimated_data.angular_velocity().x() = m_estimation_data_ptr->jv(3);
    m_estimated_data.angular_velocity().y() = m_estimation_data_ptr->jv(4);
    m_estimated_data.angular_velocity().z() = m_estimation_data_ptr->jv(5);

    m_estimated_data.plane_normal().x() = m_estimation_data_ptr->np(0);
    m_estimated_data.plane_normal().y() = m_estimation_data_ptr->np(1);
    m_estimated_data.plane_normal().z() = m_estimation_data_ptr->np(2);

    for (int i = 0; i < 6; ++i) {
        // m_estimated_data.base_wrench()[i] = m_measurement_data_ptr->estimated_base_wrench(i);
        m_estimated_data.base_wrench()[i] = m_estimation_data_ptr->bw(i);
    }

    for (int i = 0; i < 12; ++i) {
        m_estimated_data.joint_position()[i] = m_estimation_data_ptr->js(7 + i);
        m_estimated_data.joint_velocity()[i] = m_estimation_data_ptr->jv(6 + i);
        m_estimated_data.joint_torque()[i] = m_sensor_data_ptr->tau(i);
        m_estimated_data.foot_position()[i] = m_estimation_data_ptr->rP(i);
        m_estimated_data.foot_velocity()[i] = m_estimation_data_ptr->vP(i);
    }

    m_estimated_data_pub_ptr->publish(m_estimated_data);

    // write reference data and publish
    for (int i = 0; i < 4; ++i) {
        m_reference_data.contact_state()[i] = m_planner_data_ptr->cs_ref(i);
        m_reference_data.contact_prob()[i] = m_planner_data_ptr->pc_ref(i);
    }
    for (int i = 0; i < 12; ++i) {
        m_reference_data.contact_force()[i] = m_measurement_data_ptr->desired_contact_force(i);
    }
    m_reference_data.base_position().x() = m_planner_data_ptr->x(0);
    m_reference_data.base_position().y() = m_planner_data_ptr->x(1);
    m_reference_data.base_position().z() = m_planner_data_ptr->x(2);

    m_reference_data.base_orientation().x() = m_planner_data_ptr->x(3);
    m_reference_data.base_orientation().y() = m_planner_data_ptr->x(4);
    m_reference_data.base_orientation().z() = m_planner_data_ptr->x(5);
    m_reference_data.base_orientation().w() = m_planner_data_ptr->x(6);

    m_reference_data.linear_velocity().x() = m_planner_data_ptr->xd(0);
    m_reference_data.linear_velocity().y() = m_planner_data_ptr->xd(1);
    m_reference_data.linear_velocity().z() = m_planner_data_ptr->xd(2);

    m_reference_data.angular_velocity().x() = m_planner_data_ptr->xd(3);
    m_reference_data.angular_velocity().y() = m_planner_data_ptr->xd(4);
    m_reference_data.angular_velocity().z() = m_planner_data_ptr->xd(5);

    for (int i = 0; i < 6; ++i) {
        m_reference_data.base_wrench()[i] = m_measurement_data_ptr->desired_base_wrench(i);
    }

    for (int i = 0; i < 12; ++i) {
        m_reference_data.joint_position()[i] = m_joint_command_data_ptr->q(i);
        m_reference_data.joint_velocity()[i] = m_joint_command_data_ptr->qd(i);
        m_reference_data.joint_torque()[i] = m_joint_command_data_ptr->tau(i);
        m_reference_data.foot_position()[i] = m_planner_data_ptr->x(7 + i);
        m_reference_data.foot_velocity()[i] = m_planner_data_ptr->xd(6 + i);
    }

    m_reference_data_pub_ptr->publish(m_reference_data);

    m_solver_stats.iters() = m_measurement_data_ptr->solver_data.iters;
    m_solver_stats.max_iters() = m_measurement_data_ptr->solver_data.max_iters;
    for (int i = 0; i < 6; ++i) {
        m_solver_stats.residual()[i] = m_measurement_data_ptr->solver_data.residual(i);
    }
    for (int i = 0; i < 4; ++i) {
        m_solver_stats.constraint_violation()[i] = m_measurement_data_ptr->solver_data.constraint_violation(i);
    }
    m_solver_stats.time_ms() = m_measurement_data_ptr->solver_data.time_ms;

    m_solver_stats_pub_ptr->publish(m_solver_stats);

    m_base_err.x() = m_measurement_data_ptr->base_orientation_error(0);
    m_base_err.y() = m_measurement_data_ptr->base_orientation_error(1);
    m_base_err.z() = m_measurement_data_ptr->base_orientation_error(2);

    m_base_err_pub_ptr->publish(m_base_err);
}

void QuadDDSComm::write_sim_measurement_data() {
    if (m_measurement_data_ptr == NULL || m_ground_truth_data_pub_ptr == NULL) {
        return;
    }
    
    m_ground_truth_data.base_position().x() = m_measurement_data_ptr -> base_position(0);
    m_ground_truth_data.base_position().y() = m_measurement_data_ptr -> base_position(1);
    m_ground_truth_data.base_position().z() = m_measurement_data_ptr -> base_position(2);

    m_ground_truth_data.linear_velocity().x() = m_measurement_data_ptr -> base_velocity.linear(0);
    m_ground_truth_data.linear_velocity().y() = m_measurement_data_ptr -> base_velocity.linear(1);
    m_ground_truth_data.linear_velocity().z() = m_measurement_data_ptr -> base_velocity.linear(2);

    m_ground_truth_data.angular_velocity().x() = m_measurement_data_ptr -> base_velocity.angular(0);
    m_ground_truth_data.angular_velocity().y() = m_measurement_data_ptr -> base_velocity.angular(1);
    m_ground_truth_data.angular_velocity().z() = m_measurement_data_ptr -> base_velocity.angular(2);

    for (int i = 0; i < 12; ++i) {    vec3 obstacle_B = vec3::Zero();
    obstacle_B = pinocchio::Ry(M_PI) * vec3(obstacle_coords.x(), obstacle_coords.y(), obstacle_coords.z());

    m_sensor_data_ptr -> obstacle_coords = obstacle_B;
        m_ground_truth_data.contact_force()[i] = m_measurement_data_ptr -> contact_force(i);
    }

    m_ground_truth_data_pub_ptr -> publish(m_ground_truth_data);
}

void QuadDDSComm::write_joint_command() {
    if (m_joint_command_data_ptr == NULL || !m_is_executor_ready) {
        return;
    }
    for (int i = 0; i < 12; ++i) {
        m_joint_command_msg.q()[i] = m_joint_command_data_ptr->q(i);
        m_joint_command_msg.dq()[i] = m_joint_command_data_ptr->qd(i);
        m_joint_command_msg.kp()[i] = m_joint_command_data_ptr->kp(i);
        m_joint_command_msg.kd()[i] = m_joint_command_data_ptr->kd(i);
        m_joint_command_msg.tau()[i] = m_joint_command_data_ptr->tau(i);
    }

    m_joint_cmd_pub_ptr -> publish(m_joint_command_msg);
}

void QuadDDSComm::step() {
    if (m_mode == DATA_ACCESS_MODE::PLANT) {
        // plant_time.data() = m_plant_time;
        // m_plant_time_pub_ptr -> publish(plant_time);
        // get_low_cmd_data();
        write_sensor_data();
        if (publish_measurements) {
            write_sim_measurement_data();
        }
    } else if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
        write_joint_command();
        if (publish_measurements) {
            write_measurement_data();
        }
    }
}

void QuadDDSComm::run() {
    if (m_loop_rate == 0) {
        std::cout << "Communication rate not set. Defaulting to 500 Hz\n";
        m_loop_rate = 500;
        m_dt = 1./m_loop_rate;
    }
    int delay_ms = int(m_dt * 1e3);

    // std::cout << "Waiting for sensor data (thread block mode)...\n";
    // while (!m_sensor_data_sub_ptr -> messageReceived()) {
    //     m_communication_ready = true;
    //     std::this_thread::sleep_for(std::chrono::milliseconds(20));
    // }
    // std::cout << "Sensor data update received...\n";

    // Make the termination more sound since it will be running on a different thread. Break the loop automatically when communication "ends".
    while (!terminated()) {
        curr_time = get_wall_time_seconds(m_startTimePoint);
        // if ((curr_time - prev_time) > 1. / m_loop_rate) {
        //     step();
        //     prev_time = curr_time;
        // }
        step();
        delay_ms = int(1e3 * (m_dt - get_wall_time_seconds(m_startTimePoint) + curr_time));
        // Sleep is not a good way to do this. Gaali dete log aisa krne pr (according to stackoverflow)
        // This actually might be a good way to do this. The sleep function is quite accurate now. yield() instead of sleep() might also be a choice.
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
}

void QuadDDSComm::start_thread() {
    m_thread = std::thread(&QuadDDSComm::run, this);
    m_thread.detach();
}
