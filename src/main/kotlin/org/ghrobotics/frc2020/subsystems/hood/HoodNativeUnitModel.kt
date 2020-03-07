/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.hood

import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.operations.times

/**
 * Represents the native unit model for the hood.
 */
class HoodNativeUnitModel(
    private val x1: SIUnit<NativeUnit>,
    private val y1: SIUnit<Radian>,
    private val x2: SIUnit<NativeUnit>,
    private val y2: SIUnit<Radian>
) : NativeUnitModel<Radian>() {

    private val m = (y2 - y1) / (x2 - x1)
    private val b = y2 - x2 * m

    override fun fromNativeUnitPosition(nativeUnits: SIUnit<NativeUnit>): SIUnit<Radian> {
        return m * nativeUnits + b
    }

    override fun toNativeUnitPosition(modelledUnit: SIUnit<Radian>): SIUnit<NativeUnit> {
        return (modelledUnit - b) / m
    }

    override fun fromNativeUnitVelocity(nativeUnitVelocity: SIUnit<NativeUnitVelocity>): SIUnit<Velocity<Radian>> {
        return SIUnit((m * nativeUnitVelocity).value)
    }

    override fun toNativeUnitVelocity(modelledUnitVelocity: SIUnit<Velocity<Radian>>): SIUnit<NativeUnitVelocity> {
        return SIUnit((modelledUnitVelocity / m).value)
    }
}
