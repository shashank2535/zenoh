#include <zenoh.hxx>
#include <string>
#include <iostream>
#include <thread>
#include<array>
#include<cstdint>
#include "getargs.hxx"
//#include <zenoh.hpp>
using namespace zenoh;
using namespace zenoh::ext;

typedef std::array<int, 4> int4;
typedef std::array<double, 2> vec2;
typedef std::array<double, 3> vec3;
typedef std::array<double, 4> vec4;
typedef std::array<double, 6> vec6;
typedef std::array<double, 7> vec7;
typedef std::array<double, 12> vec12;
typedef std::array<double, 18> vec18;
typedef std::array<double, 19> vec19;
typedef std::array<std::array<double, 3>, 3> mat3x3;
typedef std::array<std::array<double, 4>, 4> mat4x4;
typedef std::array<std::array<double, 6>, 6> mat6x6;
typedef std::array<std::array<double, 8>, 8> mat8x8;
typedef std::array<std::array<double, 12>, 12> mat12x12;
typedef std::array<std::array<double, 12>, 18> mat12x18;
typedef std::array<std::array<double, 18>, 18> mat18x18;
typedef std::array<std::array<double, 6>, 12> mat6x12;
typedef std::array<std::array<double, 12>, 6> mat12x6;
typedef std::array<std::array<double, 6>, 18> mat6x18;
typedef std::array<std::array<double, 18>, 6> mat18x6;

template <typename T, size_t N>
std::ostream &operator<<(std::ostream &os, const std::array<T, N> &arr) {
    os << "[";
    for (size_t i = 0; i < N; ++i) {
        os << arr[i];
        if (i != N - 1) os << ", ";
    }
    os << "]";
    return os;
}

// Function to print a 2D array
template <typename T, size_t R, size_t C>
std::ostream &operator<<(std::ostream &os, const std::array<std::array<T, C>, R> &arr) {
    os << "[";
    for (const auto &row : arr) {
        os << row << ", ";
    }
    os << "]";
    return os;
}


typedef struct QuadrupedSensorData {
    vec12 q, qd;
    vec12 tau;
    vec3 a_B, w_B;
    vec3 a_W, w_W;
    vec4 quat;
    vec3 eul;
    vec3 obstacle_coords;

    QuadrupedSensorData() {
        q.fill(0);
        qd.fill(0);
        tau.fill(0);
        a_B.fill(0);
        w_B.fill(0);
        a_W.fill(0);
        w_W.fill(0);
        quat = {0, 0, 0, 1};
        eul.fill(0);
        obstacle_coords.fill(0);
    }
     friend std::ostream &operator<<(std::ostream &os, const QuadrupedSensorData &s1) {
        os << "q: " << s1.q << ", qd: " << s1.qd << ", tau: " << s1.tau
           << ", a_B: " << s1.a_B << ", w_B: " << s1.w_B << ", a_W: " << s1.a_W
           << ", w_W: " << s1.w_W << ", quat: " << s1.quat << ", eul: " << s1.eul
           << ", obstacle_coords: " << s1.obstacle_coords;
        return os;
    }
};

bool __zenoh_serialize_with_serializer(zenoh::ext::Serializer& serializer, const QuadrupedSensorData& s1, ZResult* err) {
return  zenoh::ext::detail::serialize_with_serializer(serializer, s1.q, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s1.qd, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s1.tau, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s1.a_B, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s1.w_B, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s1.a_W, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s1.w_W, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s1.quat, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s1.eul, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s1.obstacle_coords, err);
}
bool __zenoh_deserialize_with_deserializer(zenoh::ext::Deserializer& deserializer, QuadrupedSensorData& s1, ZResult* err) {
return  zenoh::ext::detail::deserialize_with_deserializer(deserializer, s1.q, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s1.qd, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s1.tau, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s1.a_B, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s1.w_B, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s1.a_W, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s1.w_W, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s1.quat, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s1.eul, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s1.obstacle_coords, err);
}
typedef struct QuadrupedEstimationData {
    vec3 rB;
    vec3 vB;
    vec12 rP;
    vec12 vP;
    int4 cs;
    vec4 pc;
    int4 is_slipping;
    vec19 js;
    vec18 jv;
    vec18 ja;
    vec3 np;
    vec6 bw;
    vec3 obstacle_coords;

    QuadrupedEstimationData() {
        rB.fill(0);
        vB.fill(0);
        rP.fill(0);
        vP.fill(0);
        cs.fill(0);
        pc.fill(0);
        is_slipping.fill(0);
        js.fill(0);
        jv.fill(0);
        ja.fill(0);
        np.fill(0);
        np[2] = 1;
        bw.fill(0);
        obstacle_coords.fill(0);
    
    }
     friend std::ostream &operator<<(std::ostream &os, const QuadrupedEstimationData &s2) {
        os << "rB: " << s2.rB << ", vB: " << s2.vB << ", rP: " << s2.rP
           << ", vP: " << s2.vP << ", cs: " << s2.cs << ", pc: " << s2.pc
           << ", is_slipping: " << s2.is_slipping << ", js: " << s2.js
           << ", jv: " << s2.jv << ", ja: " << s2.ja << ", np: " << s2.np
           << ", bw: " << s2.bw << ", obstacle_coords: " << s2.obstacle_coords;
        return os;
    }
} ;

bool __zenoh_serialize_with_serializer(zenoh::ext::Serializer& serializer, const QuadrupedEstimationData& s2, ZResult* err) {
return  zenoh::ext::detail::serialize_with_serializer(serializer, s2.rB, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s2.vB, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s2.rP, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s2.vP, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s2.cs, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s2.pc, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s2.is_slipping, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s2.js, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s2.jv, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s2.ja, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s2.np, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s2.bw, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s2.obstacle_coords, err);
}
bool __zenoh_deserialize_with_deserializer(zenoh::ext::Deserializer& deserializer, QuadrupedEstimationData& s2, ZResult* err) {
return  zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.rB, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.vB, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.rP, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.vP, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.cs, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.pc, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.is_slipping, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.js, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.jv, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.ja, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.np, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.bw, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.obstacle_coords, err);
}
typedef struct QuadrupedCommandData {
    vec12 q, qd, tau, kp, kd;

    QuadrupedCommandData() {
        q.fill(0);
        qd.fill(0);
        tau.fill(0);
        kp.fill(0);
        kd.fill(0);
    }   
     friend std::ostream &operator<<(std::ostream &os, const QuadrupedCommandData &s3) {
        os << "q: " << s3.q
           << ", qd: " << s3.qd
           << ", tau: " << s3.tau
           << ", kp: " << s3.kp
           << ", kd: " << s3.kd;
        return os;
    }
} ;

bool __zenoh_serialize_with_serializer(zenoh::ext::Serializer& serializer, const QuadrupedCommandData& s3, ZResult* err) {
return  zenoh::ext::detail::serialize_with_serializer(serializer, s3.q, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s3.qd, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s3.tau, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s3.kp, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s3.kd, err);
}
bool __zenoh_deserialize_with_deserializer(zenoh::ext::Deserializer& deserializer,QuadrupedCommandData& s3, ZResult* err) {
return  zenoh::ext::detail::deserialize_with_deserializer(deserializer, s3.q, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s3.qd, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s3.tau, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s3.kp, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s3.kd, err);
}
typedef struct QuadrupedPlannerData {
    vec19 x;
    vec18 xd, xdd;
    int4 cs_ref;
    vec4 pc_ref;
    int mode;

    QuadrupedPlannerData() {
        x.fill(0);
        xd.fill(0);
        xdd.fill(0);
        cs_ref.fill(0);
        pc_ref.fill(0);
        mode = 0;
    }
    friend std::ostream &operator<<(std::ostream &os, const QuadrupedPlannerData &s4) {
    os << "{x: " << s4.x
       << ", xd: " << s4.xd
       << ", xdd: " << s4.xdd
       << ", cs_ref: " << s4.cs_ref
       << ", pc_ref: " << s4.pc_ref
       << ", mode: " << s4.mode << "}";
    return os;
}

} ;

bool __zenoh_serialize_with_serializer(zenoh::ext::Serializer& serializer, const QuadrupedPlannerData& s4, ZResult* err) {
return  zenoh::ext::detail::serialize_with_serializer(serializer, s4.x, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s4.xd, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s4.xdd, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s4.cs_ref, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s4.pc_ref, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s4.mode, err);
}
bool __zenoh_deserialize_with_deserializer(zenoh::ext::Deserializer& deserializer, QuadrupedPlannerData& s4, ZResult* err) {
return  zenoh::ext::detail::deserialize_with_deserializer(deserializer, s4.x, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s4.xd, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s4.xdd, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s4.cs_ref, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s4.pc_ref, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s4.mode, err);
}
typedef struct QuadrupedMeasurementData {
    vec3 base_position;
    struct TotalVelocity {
        vec3 linear;
        vec3 angular;
        TotalVelocity() {
            linear.fill(0);
            angular.fill(0);
        }
    };

    struct SolverData {
        int iters = 0;
        int max_iters = 0;
        vec6 residual;
        vec4 constraint_violation;
        double time_ms = 0;

        SolverData() {
            residual.fill(0);
            constraint_violation.fill(0);
        }
    };

    TotalVelocity base_velocity;
    TotalVelocity base_acceleration;
    vec12 contact_force;
    vec12 estimated_contact_force;
    vec12 desired_contact_force;
    vec6 estimated_base_wrench;
    vec6 desired_base_wrench;
    SolverData solver_data;
    vec3 bw_ori_base_tr;
    vec3 bw_ori_base_ori;
    vec3 bw_ori_legs;
    vec3 bw_ori_full;
    vec3 base_orientation_error;

    QuadrupedMeasurementData() {
        base_position.fill(0);
        base_velocity = TotalVelocity();
        base_acceleration = TotalVelocity();
        contact_force.fill(0);
        desired_contact_force.fill(0);
        estimated_contact_force.fill(0);
        estimated_base_wrench.fill(0);
        desired_base_wrench.fill(0);
        solver_data = SolverData();
        bw_ori_base_tr.fill(0);
        bw_ori_base_ori.fill(0);
        bw_ori_legs.fill(0);
        bw_ori_full.fill(0);
        base_orientation_error.fill(0);
    }
} ;

bool __zenoh_serialize_with_serializer(zenoh::ext::Serializer& serializer, const QuadrupedMeasurementData& m, ZResult* err) {
    return zenoh::ext::detail::serialize_with_serializer(serializer, m.base_position, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.base_velocity.linear, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.base_velocity.angular, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.base_acceleration.linear, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.base_acceleration.angular, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.contact_force, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.estimated_contact_force, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.desired_contact_force, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.estimated_base_wrench, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.desired_base_wrench, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.solver_data.iters, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.solver_data.max_iters, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.solver_data.residual, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.solver_data.constraint_violation, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.solver_data.time_ms, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.bw_ori_base_tr, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.bw_ori_base_ori, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.bw_ori_legs, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.bw_ori_full, err) &&
           zenoh::ext::detail::serialize_with_serializer(serializer, m.base_orientation_error, err);
}

bool __zenoh_deserialize_with_deserializer(zenoh::ext::Deserializer& deserializer, QuadrupedMeasurementData& m, ZResult* err) {
    return zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.base_position, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.base_velocity.linear, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.base_velocity.angular, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.base_acceleration.linear, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.base_acceleration.angular, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.contact_force, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.estimated_contact_force, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.desired_contact_force, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.estimated_base_wrench, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.desired_base_wrench, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.solver_data.iters, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.solver_data.max_iters, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.solver_data.residual, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.solver_data.constraint_violation, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.solver_data.time_ms, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.bw_ori_base_tr, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.bw_ori_base_ori, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.bw_ori_legs, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.bw_ori_full, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, m.base_orientation_error, err);
}
typedef struct QuadrupedJoystickData {
    double vel_x = 0;
    double vel_y = 0;
    double vel_yaw = 0;
    double pitch = 0;
    double roll = 0;
    double yaw = 0;
    uint8_t up = 0;
    uint8_t down = 0;
    uint8_t left = 0;
    uint8_t right = 0;
    int mode = 0;

     friend std::ostream &operator<<(std::ostream &os, const QuadrupedJoystickData &s5){
    os << "{vel_x: " << s5.vel_x
       << ", vel_y: " << s5.vel_y
       << ", vel_yaw: " << s5.vel_yaw
       << ", pitch: " << s5.pitch
       << ", roll: " << s5.roll
       << ", yaw: " << s5.yaw
       << ", up: " << static_cast<int>(s5.up)
       << ", down: " << static_cast<int>(s5.down)
       << ", left: " << static_cast<int>(s5.left)
       << ", right: " << static_cast<int>(s5.right)
       << ", mode: " << s5.mode << "}";
    return os;
}

};
bool __zenoh_serialize_with_serializer(zenoh::ext::Serializer& serializer, const QuadrupedJoystickData& s5, ZResult* err) {
return  zenoh::ext::detail::serialize_with_serializer(serializer, s5.vel_x, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s5.vel_y, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s5.vel_yaw, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s5.pitch, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s5.roll, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s5.yaw, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s5.up, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s5.down, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s5.left, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s5.right, err)&&
        zenoh::ext::detail::serialize_with_serializer(serializer, s5.mode, err);
}
bool __zenoh_deserialize_with_deserializer(zenoh::ext::Deserializer& deserializer, QuadrupedJoystickData &s5, ZResult* err) {
return  zenoh::ext::detail::deserialize_with_deserializer(deserializer, s5.vel_x, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s5.vel_y, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s5.vel_yaw, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s5.pitch, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s5.roll, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s5.yaw, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s5.up, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s5.down, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s5.left, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s5.right, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s5.mode, err);
}

int main(int argc, char **argv) {

    QuadrupedSensorData s1;
    QuadrupedEstimationData s2;
    QuadrupedCommandData s3;
    QuadrupedPlannerData s4;
    QuadrupedMeasurementData m;
    QuadrupedJoystickData s5;

        Bytes b1 = ext::serialize(s1);
       
         Bytes b2 = ext::serialize(s2);

         Bytes b3 = ext::serialize(s3);
  
         Bytes b4 = ext::serialize(s4);
    
         Bytes b5 = ext::serialize(m);
   
         Bytes b6 = ext::serialize(s5);
     std::cout << "QuadrupedSensorData: " << s1 << std::endl;
    std::cout << "QuadrupedEstimationData: " << s2 << std::endl;
    std::cout << "QuadrupedCommandData:  " << s3 << std::endl;
    std::cout << "QuadrupedPlannerData: " << s4 << std::endl;
    std::cout << " QuadrupedJoystickData : " << s5 << std::endl;

    QuadrupedSensorData s11 =ext::deserialize<QuadrupedSensorData>(b1);
    QuadrupedEstimationData s21 =ext::deserialize<QuadrupedEstimationData>(b2);
    QuadrupedCommandData s31=ext::deserialize<QuadrupedCommandData>(b3);
    QuadrupedPlannerData s41=ext::deserialize<QuadrupedPlannerData>(b4);
    QuadrupedMeasurementData m1=ext::deserialize<QuadrupedMeasurementData>(b5);
    QuadrupedJoystickData s51=ext::deserialize<QuadrupedJoystickData>(b6);
    
    std::cout << "QuadrupedSensorData: " << s11 << std::endl;
    std::cout << "QuadrupedEstimationData: " << s21 << std::endl;
    std::cout << "QuadrupedCommandData:  " << s31 << std::endl;
    std::cout << "QuadrupedPlannerData: " << s41 << std::endl;
    //std::cout << " QuadrupedJoystickData : " << s5 << std::endl;

}