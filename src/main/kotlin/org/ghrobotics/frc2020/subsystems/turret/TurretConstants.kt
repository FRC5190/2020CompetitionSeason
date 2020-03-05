package org.ghrobotics.frc2020.subsystems.turret

import org.ghrobotics.frc2020.kIsRaceRobot
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds

object TurretConstants {
    const val kTurretId = 5
    const val kHallEffectSensorId = 4

    val kGearRatio = 124.0 / 16.0 * if (kIsRaceRobot) 15.0 else 12.0

    val kNativeUnitModel =
        NativeUnitRotationModel(kGearRatio.nativeUnits)
    val kAcceptableRange = (-70).degrees..290.degrees

    val kZeroLocation = 81.78.degrees

    val kAlignDelay = 0.25.seconds
    val kDefaultJogAmount = 1.degrees

    val kS = 0.0.volts

    const val kP = 1.10E-5
    const val kF = 1.041667E-4

    val kMaxVelocity = 720.degrees / 1.seconds
    val kMaxAcceleration = 560.degrees / 1.seconds / 1.seconds

    val kBadTurretOffset = 3.0.degrees

    val kTurretRelativeToRobotCenter =
        Translation2d((-8).inches, 0.inches)
}