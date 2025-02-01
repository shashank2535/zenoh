#pragma once

#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<int, 4, 1> int4;
typedef Eigen::Matrix<double, 2, 1> vec2;
typedef Eigen::Matrix<double, 3, 1> vec3;
typedef Eigen::Matrix<double, 4, 1> vec4;
typedef Eigen::Matrix<double, 6, 1> vec6;
typedef Eigen::Matrix<double, 7, 1> vec7;
typedef Eigen::Matrix<double, 12, 1> vec12;
typedef Eigen::Matrix<double, 18, 1> vec18;
typedef Eigen::Matrix<double, 19, 1> vec19;
typedef Eigen::Matrix<double, 3, 3> mat3x3;
typedef Eigen::Matrix<double, 4, 4> mat4x4;
typedef Eigen::Matrix<double, 6, 6> mat6x6;
typedef Eigen::Matrix<double, 8, 8> mat8x8;
typedef Eigen::Matrix<double, 12, 12> mat12x12;
typedef Eigen::Matrix<double, 12, 18> mat12x18;
typedef Eigen::Matrix<double, 18, 18> mat18x18;
typedef Eigen::Matrix<double, 6, 12> mat6x12;
typedef Eigen::Matrix<double, 12, 6> mat12x6;
typedef Eigen::Matrix<double, 6, 18> mat6x18;
typedef Eigen::Matrix<double, 18, 6> mat18x6;

typedef struct QuadrupedSensorData {
    vec12 q, qd;
    vec12 tau;
    vec3 a_B, w_B;
    vec3 a_W, w_W;
    vec4 quat;
    vec3 eul;
    // obstacle coordinates
    vec3 obstacle_coords;
    QuadrupedSensorData() {
        q = vec12::Zero();
        qd = vec12::Zero();
        tau = vec12::Zero();
        a_B = vec3::Zero();
        w_B = vec3::Zero();
        a_W = vec3::Zero();
        w_W = vec3::Zero();
        quat = vec4(0, 0, 0, 1);
        eul = vec3::Zero();
        obstacle_coords = vec3::Zero();
    }

    void copy(const QuadrupedSensorData& sd) {
        this->q = sd.q;
        this->qd = sd.qd;
        this->tau = sd.tau;
        this->a_B = sd.a_B;
        this->w_B = sd.w_B;
        this->a_W = sd.a_W;
        this->w_W = sd.w_W;
        this->quat = sd.quat;
        this->eul = sd.eul;
        this->obstacle_coords = sd.obstacle_coords;
    }
} QuadrupedSensorData;

typedef struct QuadrupedEstimationData {
    // Base position, velocity and feet position
    vec3 rB;
    vec3 vB;
    vec12 rP;
    vec12 vP;
    // contact states and probability
    int4 cs;
    vec4 pc;
    int4 is_slipping;
    // vec4 cs_time; // time of contact detection
    // Joint state (pos), velocity, acceleration
    vec19 js;
    vec18 jv;
    vec18 ja;
    // Plane normal
    vec3 np;
    // base wrench
    vec6 bw;
    // obstacle coordinates
    vec3 obstacle_coords;
    QuadrupedEstimationData() {
        rB = vec3::Zero();
        vB = vec3::Zero();
        rP = vec12::Zero();
        vP = vec12::Zero();
        cs = int4::Zero();
        pc = vec4::Zero();
        is_slipping = int4::Zero();
        js = vec19::Zero();
        jv = vec18::Zero();
        ja = vec18::Zero();
        np = vec3::Zero();
        bw = vec6::Zero();
        np(2) = 1;
        obstacle_coords = vec3::Zero();
    }

    void copy(const QuadrupedEstimationData& est) {
        this->rB = est.rB;
        this->vB = est.vB;
        this->rP = est.rP;
        this->vP = est.vP;
        this->cs = est.cs;
        this->pc = est.pc;
        this->is_slipping = est.is_slipping;
        this->js = est.js;
        this->jv = est.jv;
        this->ja = est.ja;
        this->np = est.np;
        this->bw = est.bw;
        this->obstacle_coords = est.obstacle_coords;
    }
} QuadrupedEstimationData;

typedef struct QuadrupedCommandData {
    vec12 q, qd, tau, kp, kd;
    QuadrupedCommandData() {
        q = vec12::Zero();
        qd = vec12::Zero();
        tau = vec12::Zero();
        kp = vec12::Zero();
        kd = vec12::Zero();
    }
    void copy(const QuadrupedCommandData& cmd) {
        this->q = cmd.q;
        this->qd = cmd.qd;
        this->tau = cmd.tau;
        this->kp = cmd.kp;
        this->kd = cmd.kd;
    }
} QuadrupedCommandData;

typedef struct QuadrupedPlannerData {
    vec19 x;
    vec18 xd, xdd;
    int4 cs_ref;
    vec4 pc_ref;
    int mode;
    QuadrupedPlannerData() {
        x = vec19::Zero();
        xd = vec18::Zero();
        xdd = vec18::Zero();
        cs_ref = int4::Zero();
        pc_ref = vec4::Zero();
        mode = 0;
    }
} QuadrupedPlannerData;

typedef struct QuadrupedMeasurementData {
    vec3 base_position;
    // A better name might be PluckerVector?
    struct TotalVelocity {
        vec3 linear;
        vec3 angular;
        TotalVelocity() {
            linear = vec3::Zero();
            angular = vec3::Zero();
        }
        void operator=(const TotalVelocity& v) {
            this->linear = v.linear;
            this->angular = v.angular;
        }
    };
    struct SolverData {
        int iters = 0;
        int max_iters = 0;
        vec6 residual = vec6::Zero();
        vec4 constraint_violation = vec4::Zero();
        double time_ms = 0;
        void operator=(const SolverData& s) {
            this->iters = s.iters;
            this->max_iters = s.max_iters;
            this->residual = s.residual;
            this->constraint_violation = s.constraint_violation;
            this->time_ms = s.time_ms;
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
        base_position = vec3::Zero();
        base_velocity = TotalVelocity();
        base_acceleration = TotalVelocity();
        contact_force = vec12::Zero();
        desired_contact_force = vec12::Zero();
        estimated_contact_force = vec12::Zero();
        estimated_base_wrench = vec6::Zero();
        desired_base_wrench = vec6::Zero();
        solver_data = SolverData();
        bw_ori_base_tr = vec3::Zero();
        bw_ori_base_ori = vec3::Zero();
        bw_ori_legs = vec3::Zero();
        bw_ori_full = vec3::Zero();
        base_orientation_error = vec3::Zero();
    }
    void copy(const QuadrupedMeasurementData& m) {
        this->base_position = m.base_position;
        this->base_velocity.linear = m.base_velocity.linear;
        this->base_velocity.angular = m.base_velocity.angular;
        this->base_acceleration.linear = m.base_acceleration.linear;
        this->base_acceleration.angular = m.base_acceleration.angular;
        this->desired_contact_force = m.desired_contact_force;
        this->contact_force = m.contact_force;
        this->estimated_contact_force = m.estimated_contact_force;
        this->estimated_base_wrench = m.estimated_base_wrench;
        this->solver_data = m.solver_data;
        this->bw_ori_base_tr = m.bw_ori_base_tr;
        this->bw_ori_base_ori = m.bw_ori_base_ori;
        this->bw_ori_legs = m.bw_ori_legs;
        this->bw_ori_full = m.bw_ori_full;
        this->base_orientation_error = m.base_orientation_error;
    }
} QuadrupedMeasurementData;

// typedef struct QuadrupedJoystickData {
//     double ly = 0;
//     double lx = 0;
//     double rx = 0;
//     double ry = 0;
//     double rt = 0;
//     double lt = 0;
//     double dx = 0;
//     double dy = 0;
//     int mode = 0; // 1: sleep, 2: fixed stand, 3: free stand, 4: move

//     void copy(const QuadrupedJoystickData& jd) {
//         this->ly = jd.ly;
//         this->lx = jd.lx;
//         this->rx = jd.rx;
//         this->ry = jd.ry;
//         this->lt = jd.lt;
//         this->rt = jd.rt;
//         this->dy = jd.dy;
//         this->dx = jd.dx;
//         this->mode = jd.mode;
//     }

//     void setZero() {
//         ly = 0;
//         lx = 0;
//         rx = 0;
//         ry = 0;
//         rt = 0; Consider a tank containing mercury, water, benzene, and, air as shown in Fig.1 below.
//         lt = 0;
//         dx = 0;
//         dy = 0;
//         mode = 0;
//     }

// } QuadrupedJoystickData;

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
    void copy(const QuadrupedJoystickData& jd) {
        this->vel_x = jd.vel_x;
        this->vel_y = jd.vel_y;
        this->vel_yaw = jd.vel_yaw;
        this->pitch = jd.pitch;
        this->roll = jd.roll;
        this->yaw = jd.yaw;
        this->up = jd.up;
        this->down = jd.down;
        this->left = jd.left;
        this->riFind the air pressure (gage). If an opening is made at the top of the tank, find the
equilibrium level of the mercury in the manometerght = jd.right;
        this->mode = jd.mode;
    }

    void setZero() {
        vel_x = 0;
        vel_y = 0;
        vel_yaw = 0;
        pitch = 0;
        roll = 0;
        up = 0;
        down = 0;
        left = 0;
        right = 0;
        mode = 0;
    }
} QuadrupedJoystickData;

// Go2 remote struct
// 16b
typedef union {
  struct {
    uint8_t R1 : 1;
    uint8_t L1 : 1;
    uint8_t start : 1;
    uint8_t select : 1;
    uint8_t R2 : 1;
    uint8_t L2 : 1;
    uint8_t F1 : 1;
    uint8_t F2 : 1;
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t up : 1;
    uint8_t right : 1;
    uint8_t down : 1;
    uint8_t left : 1;
  } components;
  uint16_t value=0;
} xKeySwitchUnion;

// 40 Byte (now used 24B)
typedef struct {
  uint8_t head[2];
  xKeySwitchUnion btn;
  float lx;
  float rx;
  float ry;
  float L2;
  float ly;

  uint8_t idle[16];
} xRockerBtnDataStruct;

