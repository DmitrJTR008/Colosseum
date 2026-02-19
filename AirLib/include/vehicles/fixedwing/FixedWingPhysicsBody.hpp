// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_fixedwingphysicsbody_hpp
#define msr_airlib_fixedwingphysicsbody_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "vehicles/multirotor/RotorActuator.hpp"
#include "api/VehicleApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "physics/PhysicsBody.hpp"
#include "physics/PhysicsBodyVertex.hpp"
#include <vector>
#include <algorithm>

namespace msr
{
namespace airlib
{

    //=======================================================================
    // FixedWingPhysicsBody -- aerodynamic model for X8 SkyWalker flying wing
    //
    // The vehicle has a single pusher motor (thrust along body +X in NED)
    // and two elevons mapped from controller pitch / roll outputs.
    //
    // Forces computed every physics tick:
    //   1. Motor thrust (along body forward axis)
    //   2. Aerodynamic lift (perpendicular to velocity in body vertical plane)
    //   3. Aerodynamic drag (opposing velocity)
    //   4. Gravity (handled by physics engine, but we compute aero forces here)
    //   5. Control moments from elevon deflection
    //=======================================================================
    class FixedWingPhysicsBody : public PhysicsBody
    {
    public:
        FixedWingPhysicsBody(MultiRotorParams* params, VehicleApiBase* vehicle_api,
                              Kinematics* kinematics, Environment* environment)
            : params_(params), vehicle_api_(vehicle_api)
        {
            setName("FixedWingPhysicsBody");
            vehicle_api_->setParent(this);
            initialize(kinematics, environment);
        }

        //=== Aerodynamic constants for X8 SkyWalker ===
        struct AeroParams
        {
            // Wing geometry
            real_T wing_area = 0.75f;        // m^2
            real_T wingspan = 2.12f;          // m
            real_T aspect_ratio = 6.0f;       // AR = b^2 / S
            real_T mean_chord = 0.354f;       // S / b

            // Lift
            real_T CL0 = 0.28f;              // zero-AoA lift coefficient (reflex airfoil, positive camber)
            real_T CLalpha = 4.8f;            // dCL/dalpha per radian
            real_T alpha_stall_pos = 0.26f;   // ~15 deg positive stall
            real_T alpha_stall_neg = -0.18f;  // ~-10 deg negative stall
            real_T CL_max = 1.5f;
            real_T CL_min = -0.8f;

            // Drag polar  CD = CD0 + k * CL^2   where k = 1/(pi*e*AR)
            real_T CD0 = 0.032f;              // parasitic drag
            real_T oswald_e = 0.82f;          // Oswald efficiency

            // Pitching moment
            real_T Cm0 = -0.02f;              // zero-AoA moment
            real_T Cm_alpha = -0.5f;          // static pitch stability (negative = stable)
            real_T Cm_de = -1.2f;             // elevator effectiveness (elevon symmetric deflection)
            real_T Cm_q = -12.0f;             // pitch damping

            // Rolling moment
            real_T Cl_da = 0.18f;             // aileron effectiveness (elevon differential deflection)
            real_T Cl_p = -0.5f;              // roll damping

            // Yawing moment (flying wing has very limited yaw authority)
            real_T Cn_r = -0.04f;             // yaw damping
            real_T Cn_da = -0.005f;           // adverse yaw from ailerons

            // Max elevon deflection in radians (~25 deg)
            real_T max_elevon_deflection = 0.436f;

            // Thrust parameters for pusher motor
            real_T max_thrust = 15.0f;        // Newtons at full throttle
        };

        //--- UpdatableState implementation ---
        virtual void resetImplementation() override
        {
            PhysicsBody::resetImplementation();

            // Reset aerodynamic state
            throttle_signal_ = 0;
            elevon_left_ = 0;
            elevon_right_ = 0;
            last_wrench_ = Wrench::zero();

            // Reset sensors
            resetSensors();
        }

        virtual void update() override
        {
            // We intentionally do NOT call PhysicsBody::update() because that
            // iterates wrenchVertices (rotors) which we don't use for aero forces.
            // Instead we compute aerodynamic wrench directly.
            UpdatableObject::update();

            // Read control signals from API/controller
            // Channel 0: throttle (0..1)
            // Channel 1: pitch command (-1..1)  -> symmetric elevon
            // Channel 2: roll command  (-1..1)  -> differential elevon
            if (vehicle_api_) {
                // For the fixed wing, getActuation returns 3 signals:
                //   index 0 -> throttle
                //   index 1 -> elevator (pitch, symmetric elevon)
                //   index 2 -> aileron  (roll, differential elevon)
                throttle_signal_ = Utils::clip(vehicle_api_->getActuation(0), 0.0f, 1.0f);

                // Elevon deflections: controller outputs are in [-1, 1]
                real_T elevator_cmd = Utils::clip(vehicle_api_->getActuation(1), -1.0f, 1.0f);
                real_T aileron_cmd = Utils::clip(vehicle_api_->getActuation(2), -1.0f, 1.0f);

                // Mix to left/right elevon deflections
                elevon_left_ = (elevator_cmd + aileron_cmd) * aero_.max_elevon_deflection;
                elevon_right_ = (elevator_cmd - aileron_cmd) * aero_.max_elevon_deflection;
            }

            // Compute and set the wrench (forces and torques)
            last_wrench_ = computeAeroWrench();
            setWrench(last_wrench_);
        }

        virtual void reportState(StateReporter& reporter) override
        {
            PhysicsBody::reportState(reporter);

            reportSensors(*params_, reporter);

            reporter.startHeading("FixedWing", 1);
            reporter.writeValue("Throttle", throttle_signal_);
            reporter.writeValue("ElevonL", elevon_left_);
            reporter.writeValue("ElevonR", elevon_right_);
            reporter.writeValue("AoA_deg", Utils::radiansToDegrees(last_alpha_));
            reporter.writeValue("Airspeed", last_airspeed_);
            reporter.endHeading(false, 1);
        }

        //--- Kinematics update (called by physics engine each tick) ---
        virtual void updateKinematics(const Kinematics::State& kinematics) override
        {
            PhysicsBody::updateKinematics(kinematics);
            updateSensorsAndController();
        }

        virtual void updateKinematics() override
        {
            PhysicsBody::updateKinematics();
            updateSensorsAndController();
        }

        //--- PhysicsBody interface ---
        // We return 0 wrench vertices because we compute all forces in update()
        // directly, not through vertex-based wrench accumulation.
        virtual uint wrenchVertexCount() const override
        {
            return 0;
        }

        virtual uint dragVertexCount() const override
        {
            return 0; // drag is computed analytically in computeAeroWrench
        }

        virtual real_T getRestitution() const override
        {
            return params_->getParams().restitution;
        }

        virtual real_T getFriction() const override
        {
            return params_->getParams().friction;
        }

        //--- Sensor access ---
        const SensorCollection& getSensors() const
        {
            return params_->getSensors();
        }

        //--- Expose rotor-like output for compatibility with PawnSimApi ---
        // We fake a single "rotor" for the pusher motor so that the pawn
        // rendering system can still show propeller spin.
        RotorActuator::Output getRotorOutput(uint rotor_index) const
        {
            RotorActuator::Output output;
            if (rotor_index == 0) {
                output.thrust = throttle_signal_ * aero_.max_thrust;
                output.torque_scaler = 0;
                output.speed = throttle_signal_ * 600.0f; // approximate RPM visual
                output.turning_direction = RotorTurningDirection::RotorTurningDirectionCW;
                output.control_signal_filtered = throttle_signal_;
                output.control_signal_input = throttle_signal_;
            }
            else {
                output = RotorActuator::Output();
            }
            return output;
        }

        const AeroParams& getAeroParams() const { return aero_; }
        real_T getThrottleSignal() const { return throttle_signal_; }
        real_T getElevonLeft() const { return elevon_left_; }
        real_T getElevonRight() const { return elevon_right_; }
        real_T getLastAirspeed() const { return last_airspeed_; }
        real_T getLastAlpha() const { return last_alpha_; }

        virtual ~FixedWingPhysicsBody() = default;

    private:
        //=== Initialization ===
        void initialize(Kinematics* kinematics, Environment* environment)
        {
            PhysicsBody::initialize(params_->getParams().mass, params_->getParams().inertia,
                                   kinematics, environment);

            throttle_signal_ = 0;
            elevon_left_ = 0;
            elevon_right_ = 0;
            last_alpha_ = 0;
            last_airspeed_ = 0;
            last_wrench_ = Wrench::zero();

            initSensors(*params_, getKinematics(), getEnvironment());
        }

        //=== Core aerodynamic computation ===
        Wrench computeAeroWrench() const
        {
            const auto& state = getKinematics();
            const auto& env = getEnvironment().getState();

            // Get body-frame velocity
            // AirSim uses NED: X=forward(North), Y=right(East), Z=down
            Vector3r vel_world = state.twist.linear;
            Vector3r vel_body = VectorMath::transformToBodyFrame(vel_world, state.pose.orientation);

            // Airspeed = magnitude of body-frame velocity
            real_T airspeed = vel_body.norm();
            const_cast<FixedWingPhysicsBody*>(this)->last_airspeed_ = airspeed;

            // Dynamic pressure
            real_T rho = env.air_density;
            real_T q_bar = 0.5f * rho * airspeed * airspeed;

            // Angle of attack: alpha = atan2(-Vz_body, Vx_body)
            // In NED body frame: X is forward, Z is down
            // Positive alpha means nose up relative to velocity vector
            real_T alpha = 0;
            if (airspeed > 0.5f) {
                alpha = std::atan2(-vel_body.z(), vel_body.x());
            }
            const_cast<FixedWingPhysicsBody*>(this)->last_alpha_ = alpha;

            // Sideslip angle
            real_T beta = 0;
            if (airspeed > 0.5f) {
                beta = std::asin(Utils::clip(vel_body.y() / airspeed, -1.0f, 1.0f));
            }

            //--- Lift coefficient ---
            real_T CL;
            if (alpha > aero_.alpha_stall_pos) {
                // Post-stall: flat plate model with reduced lift
                CL = aero_.CL_max * std::cos(alpha - aero_.alpha_stall_pos) * 0.6f;
            }
            else if (alpha < aero_.alpha_stall_neg) {
                CL = aero_.CL_min * std::cos(alpha - aero_.alpha_stall_neg) * 0.6f;
            }
            else {
                CL = aero_.CL0 + aero_.CLalpha * alpha;
            }

            //--- Drag coefficient (drag polar) ---
            real_T k = 1.0f / (M_PIf * aero_.oswald_e * aero_.aspect_ratio);
            real_T CD = aero_.CD0 + k * CL * CL;

            //--- Forces in wind frame ---
            real_T Lift = q_bar * aero_.wing_area * CL;
            real_T Drag = q_bar * aero_.wing_area * CD;

            //--- Thrust from pusher motor (along body X axis) ---
            real_T Thrust = throttle_signal_ * aero_.max_thrust;

            // Air density ratio correction for thrust
            real_T air_density_ratio = rho / 1.225f;
            Thrust *= air_density_ratio;

            //--- Elevon deflections ---
            // Symmetric = elevator, differential = aileron
            real_T delta_e = (elevon_left_ + elevon_right_) * 0.5f;  // elevator
            real_T delta_a = (elevon_left_ - elevon_right_) * 0.5f;  // aileron

            //--- Angular velocities in body frame ---
            Vector3r omega_body = state.twist.angular; // already in body frame
            real_T p = omega_body.x(); // roll rate
            real_T q = omega_body.y(); // pitch rate
            real_T r = omega_body.z(); // yaw rate

            //--- Normalize angular rates by airspeed for moment coefficients ---
            real_T phat = 0, qhat = 0, rhat = 0;
            if (airspeed > 1.0f) {
                phat = p * aero_.wingspan / (2.0f * airspeed);
                qhat = q * aero_.mean_chord / (2.0f * airspeed);
                rhat = r * aero_.wingspan / (2.0f * airspeed);
            }

            //--- Moment coefficients ---
            real_T Cl_total = aero_.Cl_da * delta_a + aero_.Cl_p * phat;     // rolling moment
            real_T Cm_total = aero_.Cm0 + aero_.Cm_alpha * alpha
                             + aero_.Cm_de * delta_e + aero_.Cm_q * qhat;    // pitching moment
            real_T Cn_total = aero_.Cn_r * rhat + aero_.Cn_da * delta_a;     // yawing moment

            //--- Compute moments ---
            real_T L_moment = q_bar * aero_.wing_area * aero_.wingspan * Cl_total;   // roll
            real_T M_moment = q_bar * aero_.wing_area * aero_.mean_chord * Cm_total; // pitch
            real_T N_moment = q_bar * aero_.wing_area * aero_.wingspan * Cn_total;   // yaw

            //--- Transform aero forces to body frame ---
            // Lift acts perpendicular to velocity in the body XZ plane (upward = -Z in NED)
            // Drag acts opposite to velocity
            // Both are currently in wind frame, transform to body frame:
            //   Fx_body =  -D*cos(alpha) + L*sin(alpha)  (force along body X)
            //   Fz_body =  -D*sin(alpha) - L*cos(alpha)  (force along body Z, down positive)
            real_T ca = std::cos(alpha);
            real_T sa = std::sin(alpha);

            real_T Fx_aero = -Drag * ca + Lift * sa;
            real_T Fz_aero = -Drag * sa - Lift * ca;

            // Side force from sideslip (simple model)
            real_T Fy_aero = -q_bar * aero_.wing_area * 0.3f * beta;

            // Total body-frame force (aero + thrust)
            Vector3r force_body(
                Fx_aero + Thrust,  // forward
                Fy_aero,           // right
                Fz_aero            // down
            );

            // Torque in body frame
            Vector3r torque_body(
                L_moment,   // roll
                M_moment,   // pitch
                N_moment    // yaw
            );

            // Transform force from body to world frame for the physics engine
            Vector3r force_world = VectorMath::transformToWorldFrame(force_body, state.pose.orientation);

            Wrench wrench;
            wrench.force = force_world;
            wrench.torque = torque_body;  // torques stay in body frame (AirSim convention)

            return wrench;
        }

        //=== Sensor management ===
        void updateSensorsAndController()
        {
            updateSensors(*params_, getKinematics(), getEnvironment());
            vehicle_api_->update();
        }

        void reportSensors(MultiRotorParams& params, StateReporter& reporter)
        {
            params.getSensors().reportState(reporter);
        }

        void updateSensors(MultiRotorParams& params, const Kinematics::State& state, const Environment& environment)
        {
            unused(state);
            unused(environment);
            params.getSensors().update();
        }

        void initSensors(MultiRotorParams& params, const Kinematics::State& state, const Environment& environment)
        {
            params.getSensors().initialize(&state, &environment);
        }

        void resetSensors()
        {
            params_->getSensors().reset();
        }

    private:
        MultiRotorParams* params_;
        VehicleApiBase* vehicle_api_;

        AeroParams aero_;

        real_T throttle_signal_ = 0;
        real_T elevon_left_ = 0;   // radians
        real_T elevon_right_ = 0;  // radians

        real_T last_alpha_ = 0;
        real_T last_airspeed_ = 0;
        Wrench last_wrench_;
    };
}
} //namespace
#endif
