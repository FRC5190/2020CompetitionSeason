/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.hood

import com.revrobotics.CANSparkMaxLowLevel
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.motors.rev.FalconMAX

/**
 * Represents the adjustable hood on the robot.
 */
object Hood : FalconSubsystem() {
    // Motor
    private val master = FalconMAX(
        id = HoodConstants.kHoodId,
        model = HoodConstants.kNativeUnitModel,
        type = CANSparkMaxLowLevel.MotorType.kBrushless
    )

    // PeriodicIO
    private val periodicIO = PeriodicIO()

    val rawEncoder get() = periodicIO.rawEncoder
    val angle get() = periodicIO.angle

    init {
        master.controller.p = HoodConstants.kP
        master.controller.ff = HoodConstants.kF

        defaultCommand = HoodPositionCommand { HoodConstants.kAcceptableRange.endInclusive - 0.2.degrees }
    }

    override fun periodic() {
        periodicIO.rawEncoder = master.encoder.rawPosition.value
        periodicIO.angle = master.encoder.position

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> master.setNeutral()
            is Output.Percent -> master.setDutyCycle(desiredOutput.percent)
            is Output.Position -> master.setPosition(desiredOutput.angle)
        }
    }

    override fun setNeutral() {
        periodicIO.desiredOutput = Output.Nothing
    }

    fun setPercent(percent: Double) {
        periodicIO.desiredOutput = Output.Percent(percent)
    }

    fun setAngle(angle: SIUnit<Radian>) {
        periodicIO.desiredOutput = Output.Position(angle.coerceIn(HoodConstants.kAcceptableRange))
    }

    private class PeriodicIO {
        var angle: SIUnit<Radian> = 0.degrees
        var desiredOutput: Output = Output.Nothing

        var rawEncoder: Double = 0.0
    }

    sealed class Output {
        object Nothing : Output()
        class Percent(val percent: Double) : Output()
        class Position(val angle: SIUnit<Radian>) : Output()
    }
}
