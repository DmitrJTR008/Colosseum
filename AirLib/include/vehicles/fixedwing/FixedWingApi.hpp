// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_FixedWingApi_hpp
#define msr_airlib_FixedWingApi_hpp

#include "vehicles/multirotor/firmwares/simple_flight/SimpleFlightApi.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/AirSimSettings.hpp"
#include "common/Common.hpp"
#include "firmware/Firmware.hpp"
#include "vehicles/multirotor/firmwares/simple_flight/AirSimSimpleFlightBoard.hpp"
#include "vehicles/multirotor/firmwares/simple_flight/AirSimSimpleFlightCommLink.hpp"
#include "vehicles/multirotor/firmwares/simple_flight/AirSimSimpleFlightEstimator.hpp"
#include "vehicles/multirotor/firmwares/simple_flight/AirSimSimpleFlightCommon.hpp"
#include "physics/PhysicsBody.hpp"

namespace msr
{
namespace airlib
{

    //=======================================================================
    // FixedWingApi
    //
    // Extends SimpleFlightApi to provide 3-channel actuation mapping for
    // fixed-wing aircraft:
    //   Channel 0: Throttle   (0..1)   -> pusher motor
    //   Channel 1: Elevator   (-1..1)  -> pitch (symmetric elevon)
    //   Channel 2: Aileron    (-1..1)  -> roll  (differential elevon)
    //
    // The SimpleFlight cascade PID controller outputs 4 channels via the mixer:
    //   [0]=front_right, [1]=rear_left, [2]=front_left, [3]=rear_right
    //
    // For a fixed-wing, the Mixer's quad-X mixing produces motor outputs
    // that we re-interpret:
    //   - Throttle = average of all 4 motor outputs (the common-mode)
    //   - Elevator = pitch signal from controller
    //   - Aileron  = roll signal from controller
    //
    // We override getActuation() and getActuatorCount() to provide the
    // 3-channel interface that FixedWingPhysicsBody expects.
    //=======================================================================
    class FixedWingApi : public SimpleFlightApi
    {
    public:
        FixedWingApi(const MultiRotorParams* vehicle_params,
                     const AirSimSettings::VehicleSetting* vehicle_setting)
            : SimpleFlightApi(vehicle_params, vehicle_setting)
        {
        }

        virtual ~FixedWingApi() = default;

        //--- Override actuation interface ---

        virtual real_T getActuation(unsigned int channel_index) const override
        {
            // The base SimpleFlightApi has 4 motor outputs from the QuadX mixer.
            // We read the 4 motor outputs and re-map them to fixed-wing channels.
            //
            // QuadX mixer matrix (from Mixer.hpp):
            //   motor[0] = T - roll + pitch + yaw   (FRONT_R)
            //   motor[1] = T + roll - pitch + yaw   (REAR_L)
            //   motor[2] = T + roll + pitch - yaw   (FRONT_L)
            //   motor[3] = T - roll - pitch - yaw   (REAR_R)
            //
            // From these:
            //   Throttle = (m0+m1+m2+m3)/4 = T
            //   Pitch    = (m0-m1+m2-m3)/4 = pitch_component (maps to elevator)
            //   Roll     = (-m0+m1+m2-m3)/4 = roll_component (maps to aileron)

            real_T m0 = SimpleFlightApi::getActuation(0);
            real_T m1 = SimpleFlightApi::getActuation(1);
            real_T m2 = SimpleFlightApi::getActuation(2);
            real_T m3 = SimpleFlightApi::getActuation(3);

            switch (channel_index) {
            case 0: // Throttle: average of all motors (0..1)
                return (m0 + m1 + m2 + m3) / 4.0f;

            case 1: // Elevator (pitch): extract pitch component from mixer
            {
                // From mixer: pitch appears as +pitch in m0, -pitch in m1, +pitch in m2, -pitch in m3
                real_T pitch_signal = (m0 - m1 + m2 - m3) / 4.0f;
                return Utils::clip(pitch_signal * 2.0f, -1.0f, 1.0f);  // scale and clip
            }

            case 2: // Aileron (roll): extract roll component from mixer
            {
                // From mixer: roll appears as -roll in m0, +roll in m1, +roll in m2, -roll in m3
                real_T roll_signal = (-m0 + m1 + m2 - m3) / 4.0f;
                return Utils::clip(roll_signal * 2.0f, -1.0f, 1.0f);  // scale and clip
            }

            default:
                return 0;
            }
        }

        virtual size_t getActuatorCount() const override
        {
            return 3; // throttle, elevator, aileron
        }
    };
}
} //namespace
#endif
