#pragma once

#include "memory_types.hpp"
#include "utils.hpp"
#include <string>
#include <iostream>
#include <mutex>

enum DATA_ACCESS_MODE {
    PLANT,
    EXECUTOR
};
// Single communication manager for quadruped
class CommunicationManager {
public:
    CommunicationManager();
    CommunicationManager(const DATA_ACCESS_MODE& mode);
    CommunicationManager(const std::string&, const DATA_ACCESS_MODE& mode);
    ~CommunicationManager() {}

    virtual void setPlannerDataPtr(QuadrupedPlannerData* planner_data_ptr) {
        m_planner_data_ptr = planner_data_ptr;
    }
    virtual void setSensorDataPtr(QuadrupedSensorData* sensor_data_ptr) {
        m_sensor_data_ptr = sensor_data_ptr;
    }
    virtual void setCommandDataPtr(QuadrupedCommandData* command_data_ptr) {
        m_joint_command_data_ptr = command_data_ptr;
    }
    virtual void setMeasurementDataPtr(QuadrupedMeasurementData* measurement_data_ptr) {
        m_measurement_data_ptr = measurement_data_ptr;
    }
    virtual void setEstimationDataPtr(QuadrupedEstimationData* estimation_data_ptr) {
        m_estimation_data_ptr = estimation_data_ptr;
    }
    virtual void setJoystickDataPtr(QuadrupedJoystickData* joystick_data_ptr) {
        m_joystick_data_ptr = joystick_data_ptr;
    }
    virtual void setPlantTimePtr(double* time_ptr) {
        m_plant_time_ptr = time_ptr;
    }

    void writeSensorData(const QuadrupedSensorData& sensor_data);
    void writeCommandData(const QuadrupedCommandData& cmd_data);
    void writeMeasurementData(const QuadrupedMeasurementData& measure_data);
    void writeEstimationData(const QuadrupedEstimationData& est_data);
    void writeJoystickData(const QuadrupedJoystickData& joy_data);
    void writePlantTime(const double& time);
    
    void getSensorData(QuadrupedSensorData& sensor_data);
    void getCommandData(QuadrupedCommandData& cmd_data);
    void getMeasurememtData(QuadrupedMeasurementData& measure_data);
    void getEstimationData(QuadrupedEstimationData& est_data);
    void getJoystickData(QuadrupedJoystickData& joy_data);
    void getPlantTime(double& time);

    void setAccessMode(const DATA_ACCESS_MODE& mode) {
        m_mode = mode;
    }

    void stop();

    void setExecutorReady(const bool& flag = false) {
    	m_is_executor_ready = flag;
    }

    bool isReady() {
        return m_communication_ready;
    }

protected:
    QuadrupedPlannerData* m_planner_data_ptr = NULL;
    QuadrupedSensorData* m_sensor_data_ptr = NULL;
    QuadrupedCommandData* m_joint_command_data_ptr = NULL;
    QuadrupedEstimationData* m_estimation_data_ptr = NULL;
    QuadrupedMeasurementData* m_measurement_data_ptr = NULL;
    QuadrupedJoystickData* m_joystick_data_ptr = NULL;
    double* m_plant_time_ptr = NULL;

    int m_mode = DATA_ACCESS_MODE::PLANT;
    bool terminated();
    bool m_is_executor_ready = false;
    
    std::mutex m_sensor_data_mutex;

    bool m_communication_ready = false;
    bool m_joystick_is_nintendo = true;
private:
    std::string m_name;
    bool m_shutdown_node = false;
};
