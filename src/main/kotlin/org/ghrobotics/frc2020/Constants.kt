/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020

import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.volts
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
object TurretConstants {
    const val kTurretId = 5
    const val kHallEffectSensorId = 1

    const val kGearRatio = 100.0 / 20.0 * 124.0 / 18.0 * 3.0

    val kNativeUnitModel = NativeUnitRotationModel(kGearRatio.nativeUnits)
    val kAcceptableRange = (-200).degrees..200.degrees

    val kS = 0.0.volts

    const val kP = 1.50E-5
    const val kF = 1.041667E-4

    val kMaxVelocity = 720.degrees / 1.seconds
    val kMaxAcceleration = 550.degrees / 1.seconds / 1.seconds

    val kBadTurretOffset = 3.0.degrees

    val kTurretRelativeToRobotCenter = Translation2d((-6).inches, 0.inches)
}

@Suppress("MemberVisibilityCanBePrivate", "unused")
object ShooterConstants {
    const val kMasterId = 6
    const val kSlaveId = 7

    const val kHoodServoAId = 3
    const val kHoodServoBId = 4

    const val kGearRatio = 1.0 / 2.0

    val kNativeUnitModel = NativeUnitRotationModel(kGearRatio.nativeUnits)
    val kStowedHoodAngle = 10.degrees

    val kS = 0.volts

    const val kP = 0.0 // TODO Tune
    const val kF = 0.0 // TODO Find
}

object IntakeConstants {
    const val kIntakeId = 9
}

object VisionConstants {
    const val kLEDId = 0

    val kGoalHeight = 92.inches
    val kCameraHeight = 22.5.inches
    val kCameraAngle = Rotation2d.fromDegrees(15.0)

    val kCameraRelativeToTurretCenter = Pose2d(10.5.inches, 0.inches, Rotation2d())

    val kGoalFieldRelativeAngle = Rotation2d()

    val kMaxTargetTrackingLifetime = 0.5.seconds
    val kTargetTrackingDistanceErrorTolerance = 6.inches
    const val kMedianWindowSize = 10
}
