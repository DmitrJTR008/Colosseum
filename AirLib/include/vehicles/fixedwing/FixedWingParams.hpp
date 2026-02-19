// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_FixedWingParams_hpp
#define msr_airlib_vehicles_FixedWingParams_hpp

#include "vehicles/fixedwing/FixedWingApi.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr
{
namespace airlib
{

    //=======================================================================
    // FixedWingParams -- vehicle parameter configuration for X8 SkyWalker
    //
    // Inherits from MultiRotorParams so it integrates seamlessly with the
    // existing MultiRotorParamsFactory and SimModeWorldMultiRotor pipelines.
    //
    // Key differences from a multirotor config:
    //   - Single pusher rotor (rotor_count = 1) for compatibility with
    //     the RotorActuator system, but actual forces are computed by
    //     FixedWingPhysicsBody's aerodynamic model.
    //   - Mass, inertia, and body box configured for a flying wing.
    //   - The SimpleFlightApi is reused with motor_count = 3 in firmware
    //     params (throttle, elevator, aileron) so the mixer produces
    //     3 output channels that FixedWingPhysicsBody consumes.
    //=======================================================================
    class FixedWingParams : public MultiRotorParams
    {
    public:
        FixedWingParams(const AirSimSettings::VehicleSetting* vehicle_setting,
                        std::shared_ptr<const SensorFactory> sensor_factory)
            : vehicle_setting_(vehicle_setting), sensor_factory_(sensor_factory)
        {
        }

        virtual ~FixedWingParams() = default;

        virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() override
        {
            return std::unique_ptr<MultirotorApiBase>(new SimpleFlightApi(this, vehicle_setting_));
        }

    protected:
        virtual void setupParams() override
        {
            auto& params = getParams();

            setupFrameX8SkyWalker(params);
        }

        virtual const SensorFactory* getSensorFactory() const override
        {
            return sensor_factory_.get();
        }

    private:
        void setupFrameX8SkyWalker(Params& params)
        {
            //=== X8 SkyWalker physical properties ===

            // Single pusher motor at the rear -- we still define one rotor pose
            // so the RotorActuator infrastructure works, but FixedWingPhysicsBody
            // computes the actual thrust via its own aerodynamic model.
            params.rotor_count = 1;

            // Mass: X8 SkyWalker typical flying weight 2.0 - 3.5 kg
            params.mass = 2.5f;

            // Body box approximation for a flying wing:
            //   X (chord/length)  ~0.60 m
            //   Y (wingspan)      ~2.12 m  (though we use half for the body box center)
            //   Z (thickness)     ~0.12 m
            params.body_box.x() = 0.60f;
            params.body_box.y() = 2.12f;
            params.body_box.z() = 0.12f;

            // Rotor params for the single pusher motor
            // These are overridden by the FixedWingPhysicsBody anyway,
            // but we set them for the RotorActuator's internal bookkeeping.
            params.rotor_params.C_T = 0.1f;
            params.rotor_params.C_P = 0.04f;
            params.rotor_params.max_rpm = 8000.0f;
            params.rotor_params.propeller_diameter = 0.254f;  // 10-inch prop
            params.rotor_params.propeller_height = 0.01f;
            params.rotor_params.calculateMaxThrust();

            // Single pusher motor position: behind center of gravity, thrust along +X
            // Position in NED body frame: X is forward, so rear = negative X
            params.rotor_poses.clear();
            Vector3r pusher_position(-0.25f, 0, 0);     // 25 cm behind CG
            Vector3r pusher_normal(1, 0, 0);             // thrust direction: forward (+X body)
            params.rotor_poses.emplace_back(pusher_position, pusher_normal,
                                            RotorTurningDirection::RotorTurningDirectionCW);

            // Linear drag coefficient -- lower than quadrotor because flying wing is streamlined
            params.linear_drag_coefficient = 0.1f;
            params.angular_drag_coefficient = 0.05f;

            // Collision parameters
            params.restitution = 0.3f;  // less bouncy than a multirotor
            params.friction = 0.7f;     // more friction on ground (landing gear / belly)

            // Inertia matrix for a flying wing
            // Approximated as a flat plate: Ixx is small (thin), Iyy is moderate, Izz is larger
            // Using standard flat plate formulas but adjusted for actual flying wing geometry
            real_T motor_assembly_weight = 0.15f;
            real_T box_mass = params.mass - motor_assembly_weight;

            params.inertia = Matrix3x3r::Zero();
            // Ixx: rotation about X (forward) axis -- relatively small for flying wing
            params.inertia(0, 0) = box_mass / 12.0f * (params.body_box.y() * params.body_box.y()
                                   + params.body_box.z() * params.body_box.z());
            // Iyy: rotation about Y (right) axis -- pitch
            params.inertia(1, 1) = box_mass / 12.0f * (params.body_box.x() * params.body_box.x()
                                   + params.body_box.z() * params.body_box.z());
            // Izz: rotation about Z (down) axis -- yaw
            params.inertia(2, 2) = box_mass / 12.0f * (params.body_box.x() * params.body_box.x()
                                   + params.body_box.y() * params.body_box.y());

            // Add motor contribution to inertia
            const auto& motor_pos = params.rotor_poses[0].position;
            params.inertia(0, 0) += (motor_pos.y() * motor_pos.y() + motor_pos.z() * motor_pos.z()) * motor_assembly_weight;
            params.inertia(1, 1) += (motor_pos.x() * motor_pos.x() + motor_pos.z() * motor_pos.z()) * motor_assembly_weight;
            params.inertia(2, 2) += (motor_pos.x() * motor_pos.x() + motor_pos.y() * motor_pos.y()) * motor_assembly_weight;
        }

    private:
        const AirSimSettings::VehicleSetting* vehicle_setting_;
        std::shared_ptr<const SensorFactory> sensor_factory_;
    };
}
} //namespace
#endif
