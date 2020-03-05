package org.ghrobotics.frc2020.subsystems.drivetrain

import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

object DriveConstants {
    const val kLeftMasterId = 1
    const val kLeftSlaveId = 2
    const val kRightMasterId = 3
    const val kRightSlaveId = 4

    const val kPigeonId = 17

    val kWheelRadius = 3.inches
    val kTrackWidth = 27.75.inches
    val kNativeUnitModel = NativeUnitLengthModel(
        9.09.nativeUnits,
        kWheelRadius
    )

    const val kP = 0.00015
}