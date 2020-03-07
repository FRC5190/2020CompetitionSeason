/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.hood

import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

object HoodConstants {
    const val kHoodId = 15

    const val kP = 1.0E-5
    const val kF = 1.23E-4

    val kNativeUnitModel = HoodNativeUnitModel(
        0.nativeUnits, 42.61.degrees,
        (-11.166).nativeUnits, 9.degrees
    )

    val kAcceptableRange = 10.degrees..42.61.degrees
}
