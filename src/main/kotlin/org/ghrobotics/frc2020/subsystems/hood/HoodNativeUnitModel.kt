package org.ghrobotics.frc2020.subsystems.hood

import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
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
}