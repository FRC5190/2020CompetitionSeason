/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.IntakeConstants.kIntakeId
import org.ghrobotics.frc2020.commands.IntakeCommand
import org.ghrobotics.frc2020.commands.TestIntakeCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX

object Intake : FalconSubsystem() {

    val intakeMotor = FalconMAX(
        id = kIntakeId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = DefaultNativeUnitModel
    )

    private val periodicIO = PeriodicIO()
    val position get() = periodicIO.position

    private class PeriodicIO {
        var current: SIUnit<Ampere> = 0.amps
        var voltage: SIUnit<Volt> = 0.volts
        var position: SIUnit<NativeUnit> = 0.nativeUnits

        var feedforward: SIUnit<Volt> = 0.volts
        var desiredOutput: Output = Output.Nothing
    }

    override fun periodic() {
        periodicIO.voltage = intakeMotor.voltageOutput
        periodicIO.current = intakeMotor.drawnCurrent
        periodicIO.position = intakeMotor.encoder.position

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing ->
                intakeMotor.setNeutral()
            is Output.Percent ->
                intakeMotor.setDutyCycle(desiredOutput.percent, periodicIO.feedforward)
        }
    }

    sealed class Output() {
        object Nothing : Output()
        class Percent(val percent: Double) : Output()
    }

    override fun setNeutral() {
        periodicIO.desiredOutput = Output.Nothing
        periodicIO.feedforward = 0.volts
    }

    fun setPercent(percent: Double) {
        periodicIO.desiredOutput = Output.Percent(percent)
        periodicIO.feedforward = 0.volts
    }

    init {
        defaultCommand = IntakeCommand { 0.0 }
    }

    override fun checkSubsystem(): Command {
        return TestIntakeCommand().withTimeout(3.0)
    }
}
