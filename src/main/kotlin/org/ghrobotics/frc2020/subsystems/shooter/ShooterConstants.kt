package org.ghrobotics.frc2020.subsystems.shooter

import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

object ShooterConstants {
    const val kMasterId = 6
    const val kSlaveId = 7

    const val kGearRatio = 1.0 / 2.0

    val kNativeUnitModel =
        NativeUnitRotationModel(kGearRatio.nativeUnits)

    const val kS = 0.272
    const val kV = 0.01
    const val kA = 0.0

    const val kP = 0.00025
    const val kF = 0.0
}