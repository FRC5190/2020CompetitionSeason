/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds

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

    const val kP = 0.0
}

@Suppress("MemberVisibilityCanBePrivate", "unused")
object ShooterConstants {
    const val kMasterId = 6
    const val kSlaveId = 7

    val kNativeUnitModel = NativeUnitRotationModel(1.nativeUnits)

    const val kS = 0.0
    const val kV = 0.0
    const val kA = 0.0

    const val kP = 0.0
}

object IntakeConstants {
    const val kIntakeId = 9
}

@Suppress("MemberVisibilityCanBePrivate", "unused")
object TurretConstants {
    const val kTurretId = 5
    const val kHallEffectSensorId = 1

    // const val kGearRatio = 44.0
    const val kGearRatio = 100.0 / 20.0 * 124.0 / 18.0 * 4096 // For testing with 775Pro

    val kNativeUnitModel = NativeUnitRotationModel(kGearRatio.nativeUnits)

    val kAcceptableRange = (-200).degrees..200.degrees

    const val kS = 0.0
    const val kV = 0.0
    const val kA = 0.0

    const val kP = 0.0
    const val kI = 0.0
    const val kD = 0.0

    val kMaxVelocity = 360.degrees / 1.seconds
    val kMaxAcceleration = 20.degrees / 1.seconds / 1.seconds

    val kConstraints = TrapezoidProfile.Constraints(kMaxVelocity.value, kMaxAcceleration.value)
}
