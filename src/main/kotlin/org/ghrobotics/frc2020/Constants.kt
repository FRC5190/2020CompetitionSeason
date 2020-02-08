/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

@file:Suppress("MemberVisibilityCanBePrivate", "unused")

package org.ghrobotics.frc2020

import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.SlopeNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds

object ClimberConstants {
    const val kWinchMasterId = 11
    const val kWinchSlaveId = 12

    const val kPCMId = 41
    const val kExtensionPistonId = 1
    const val kWinchBrakeId = 4
}

object DriveConstants {
    const val kLeftMasterId = 1
    const val kLeftSlave1Id = 2
    const val kRightMasterId = 3
    const val kRightSlave1Id = 4

    const val kPigeonId = 17

    val kWheelRadius = 3.inches
    val kTrackWidth = 27.75.inches
    val kNativeUnitModel = NativeUnitLengthModel(7.29.nativeUnits, kWheelRadius)

    const val kP = 0.0003
}

object FeederConstants {
    const val kFeederMasterId = 10
    val kFeederRadius = 1.inches
    val kFeederUnitModel = NativeUnitLengthModel(42.nativeUnits, kFeederRadius)
}

object FortuneWheelConstants {
    // Determines the bit-depth of colors when being compared.
    // The bit-depth determines the amount of possible colors.
    // Ex: '255' would result in an 8bit color spectrum.
    const val kColorBitDepth = 9

    // Spinner
    const val kSpinnerSpeed = 0.2
    const val kSpinnerMotorId = 8
    val kSpinnerRadius = 3.inches // Radius of wheel connected to the spinner motor
    val kSpinnerUnitModel = NativeUnitLengthModel(42.nativeUnits, kSpinnerRadius) // Unit model for the spinner motor

    // Accuracy
    const val kDataAccuracy = 4
    const val kCompletion = 10
}


object HoodConstants {
    const val kServoAId = 0
    const val kServoBId = 1

    const val kEncoderAId = 2
    const val kEncoderBId = 3

    const val kP = 0.0

    const val kGearRatio = 0.5
    val kNativeUnitModel = NativeUnitRotationModel(4096.nativeUnits * kGearRatio)
}

object HookConstants {
    const val kHookId = 13
    val kHookNativeUnitModel = SlopeNativeUnitModel(1.meters, 150.nativeUnits)
}


object IntakeConstants {
    const val kIntakeId = 9
    const val kIntakeModuleId = 41
    const val kIntakePistonId = 0
}

object LEDConstants {
    const val kPort = 9
    const val kBufferSize = 60
}


object ShooterConstants {
    const val kMasterId = 6
    const val kSlaveId = 7

    const val kGearRatio = 1.0 / 2.0

    val kNativeUnitModel = NativeUnitRotationModel(kGearRatio.nativeUnits)

    val kS = 0.volts

    const val kP = 3.5E-4
    const val kF = 2.0E-4
}

object TurretConstants {
    const val kTurretId = 5
    const val kHallEffectSensorId = 1

    const val kGearRatio = 100.0 / 20.0 * 124.0 / 18.0 * 3.0

    val kNativeUnitModel = NativeUnitRotationModel(kGearRatio.nativeUnits)
    val kAcceptableRange = (-200).degrees..200.degrees

    val kAlignDelay = 0.25.seconds
    val kDefaultJogAmount = 1.degrees

    val kS = 0.0.volts

    const val kP = 2.50E-5
    const val kF = 1.041667E-4

    val kMaxVelocity = 720.degrees / 1.seconds
    val kMaxAcceleration = 560.degrees / 1.seconds / 1.seconds

    val kBadTurretOffset = 3.0.degrees

    val kTurretRelativeToRobotCenter = Translation2d((6).inches, 0.inches)
}

object VisionConstants {
    const val kLEDId = 0

    val kGoalHeight = 92.inches
    val kCameraHeight = 22.5.inches
    val kCameraAngle: Rotation2d = Rotation2d.fromDegrees(25.0)

    val kTurretToCamera = Pose2d(10.5.inches, 0.inches, Rotation2d())

    val kGoalFieldRelativeAngle = Rotation2d()

    val kMaxTargetTrackingLifetime = 1.5.seconds
    val kTargetTrackingDistanceErrorTolerance = 6.inches
    const val kMedianWindowSize = 10

    val kGoalLocation = Pose2d(54.feet, 94.66.inches, Rotation2d())
}
