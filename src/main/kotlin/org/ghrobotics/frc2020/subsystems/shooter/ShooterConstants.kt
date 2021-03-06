/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.shooter

import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

object ShooterConstants {
    const val kMasterId = 6
    const val kSlaveId = 7

    private const val kGearRatio = 1.0 / 2.0

    val kNativeUnitModel =
        NativeUnitRotationModel(kGearRatio.nativeUnits)

    const val kS = 0.128
    const val kV = 0.0106
    const val kA = 0.0

    const val kP = 0.00034
    const val kF = 0.0
}
