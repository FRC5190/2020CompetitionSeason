package org.ghrobotics.frc2020.subsystems.fortunewheel

import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds

object FortuneWheelConstants {
    // Determines the bit-depth of colors when being compared.
    const val kColorBitDepth = 8

    // Spinner
    const val kSpinnerMotorId = 8
    val kSpinnerRadius = 1.5.inches // Radius of wheel connected to the spinner motor
    val kSpinnerUnitModel = NativeUnitLengthModel(
        5.nativeUnits,
        kSpinnerRadius
    ) // Unit model for the spinner motor

    const val kFortuneWheelPistonId = 5

    // Fortunewheel
    val kColorDistance = 10.inches

    // Controller
    const val kP = 3E-5
    const val kF = 0.0
    val kMaxVelocity = 100.inches / 1.seconds
    val kMaxAcceleration = 200.inches / 1.seconds / 1.seconds
}