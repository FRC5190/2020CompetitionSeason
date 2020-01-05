/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020

import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.operations.times

/**
 * Contains constants for the drivetrain.
 */
@Suppress("MemberVisibilityCanBePrivate", "unused")
object DriveConstants {
    const val kLeftMasterId = 1
    const val kLeftSlave1Id = 2
    const val kRightMasterId = 3
    const val kRightSlave1Id = 4

    const val kPigeonId = 17

    val kWheelRadius = 3.inches
    val kTrackWidth = 27.75.inches
    val kNativeUnitModel = NativeUnitLengthModel(9.09.nativeUnits, kWheelRadius)

    val kP = 0.0
}

object IntakeConstants {
    const val kIntakeId = 9
}

object FortuneWheelConstants {
    const val kFortuneMotorId = 5

    val kFortuneWheelSpeed = 30.0 * 360.degrees.velocity // RPM to spin fortune wheel at
    val kSpinnerWheelRadius = 3.inches // Radius of wheel connected to the motor
    val kFortuneWheelRadius = 16.inches // Distance from the center of the fortune wheel to our contact point
    val kSpinnerUnitModel = NativeUnitLengthModel(42.nativeUnits, kSpinnerWheelRadius)
}
