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
import org.ghrobotics.frc2020.subsystems.hood.HoodNativeUnitModel
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Transform2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds

object ClimberConstants {
    const val kWinchMasterId = 11
    const val kWinchSlaveId = 12

    const val kPCMId = 0
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
    val kNativeUnitModel = NativeUnitLengthModel(9.09.nativeUnits, kWheelRadius)

    const val kP = 0.0004
}

object FeederConstants {
    const val kFeederId = 10
    const val kBridgeId = 14

    const val kPCMId = 0
    const val kExitPistonId = 2

    const val kIntakeSensorId = 3
    const val kExitSensorId = 1

    val kFeederRadius = 1.inches
    val kFeederUnitModel = NativeUnitLengthModel(42.nativeUnits, kFeederRadius)

    val kCurrentLimit = 25.amps
}

object HoodConstants {
    const val kServoAId = 0
    const val kServoBId = 1

    const val kEncoderAId = 0
    const val kEncoderBId = 1

    const val kP = 8.2

    val kNativeUnitModel = HoodNativeUnitModel(
        0.nativeUnits, 42.61.degrees,
        (-690).nativeUnits, 13.degrees
    )

    val kBadHoodOffset = (-5).degrees
    val kAcceptableRange = 10.degrees..42.61.degrees
}

object HookConstants {
    const val kHookId = 13
    const val kGearRatio = 50.0

    val kWheelRadius = 2.inches
    val kNativeUnitModel = NativeUnitLengthModel(kGearRatio.nativeUnits, kWheelRadius)
}

object IntakeConstants {
    const val kMasterId = 9

    const val kIntakeModuleId = 0
    const val kIntakePistonId = 0

    val kCurrentLimit = 25.amps
}

object LEDConstants {
    const val kPort = 2
    const val kBufferSize = 72
}

object ShooterConstants {
    const val kMasterId = 6
    const val kSlaveId = 7

    const val kGearRatio = 1.0 / 2.0

    val kNativeUnitModel = NativeUnitRotationModel(kGearRatio.nativeUnits)

    const val kS = 0.112
    const val kV = 0.00986
    const val kA = 0.0

    const val kP = 0.00051
    const val kF = 0.0
}

object FortuneWheelConstants {
    // Determines the bit-depth of colors when being compared.
    const val kColorBitDepth = 8

    // Spinner
    const val kSpinnerMotorId = 8
    val kSpinnerRadius = 1.5.inches // Radius of wheel connected to the spinner motor
    val kSpinnerUnitModel = NativeUnitLengthModel(5.nativeUnits, kSpinnerRadius) // Unit model for the spinner motor

    // Fortunewheel
    val kColorDistance = 10.inches

    // Controller
    const val kP = 3E-5
    const val kF = 0.0
    val kMaxVelocity = 100.inches / 1.seconds
    val kMaxAcceleration = 200.inches / 1.seconds / 1.seconds
}

object TurretConstants {
    const val kTurretId = 5
    const val kHallEffectSensorId = 4

    const val kGearRatio = 124.0 / 16.0 * 12.0

    val kNativeUnitModel = NativeUnitRotationModel(kGearRatio.nativeUnits)
    val kAcceptableRange = (-70).degrees..290.degrees

    val kAlignDelay = 0.25.seconds
    val kDefaultJogAmount = 1.degrees

    val kS = 0.0.volts

    const val kP = 1.50E-5
    const val kF = 1.041667E-4

    val kMaxVelocity = 720.degrees / 1.seconds
    val kMaxAcceleration = 560.degrees / 1.seconds / 1.seconds

    val kBadTurretOffset = 3.0.degrees

    val kTurretRelativeToRobotCenter = Translation2d((-8).inches, 0.inches)
}

object VisionConstants {
    const val kLEDId = 2

    val kGoalHeight = 104.inches
    val kCameraHeight = 23.75.inches
    val kCameraAngle: Rotation2d = Rotation2d.fromDegrees(23.5)

    val kTurretToCamera = Pose2d(10.5.inches, 0.inches, Rotation2d.fromDegrees(2.0))

    val kGoalFieldRelativeAngle = Rotation2d()

    val kMaxTargetTrackingLifetime = 1.5.seconds
    val kTargetTrackingDistanceErrorTolerance = 25.inches
    const val kMedianWindowSize = 15

    val kGoalLocation = Pose2d(54.feet, 94.66.inches, Rotation2d())
    val kOuterToInnerGoalTransform = Transform2d(27.inches, 0.inches, Rotation2d())
}

object ForkConstants {
    const val kPCMId = 41
    const val kForkId = 2
}
