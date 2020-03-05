package org.ghrobotics.frc2020.subsystems.climber

import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.milli

object ClimberConstants {
    const val kWinchMasterId = 11
    const val kWinchSlaveId = 12

    const val kExtensionPistonId = 1
    const val kWinchBrakeId = 4

    val kContinuousCurrentLimit = 38.amps
    val kPeakCurrentLimit = 70.amps
    val kPeakCurrentLimitDuration = 250.milli.seconds
}