package org.ghrobotics.frc2020.subsystems.hood

import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

object HoodConstants {
    const val kHoodId = 15

    const val kP = 0.0
    const val kF = 0.0

    val kNativeUnitModel = HoodNativeUnitModel(
        0.nativeUnits, 42.61.degrees,
        (-784).nativeUnits, 9.degrees
    )

    val kAcceptableRange = 10.degrees..42.61.degrees
}