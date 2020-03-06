/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.intake

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.frc2020.subsystems.intake.IntakeConstants.kIntakePistonId
import org.ghrobotics.frc2020.kPCMId
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
import org.ghrobotics.lib.utils.isConnected

object Intake : FalconSubsystem() {

    private val intakeMaster = FalconMAX(
        id = IntakeConstants.kIntakeId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = DefaultNativeUnitModel
    )

    private val intakePiston = Solenoid(kPCMId, kIntakePistonId)

    private val periodicIO = PeriodicIO()
    val position get() = periodicIO.position

    // Connection Status
    private var isConnected = false

    override fun lateInit() {
        isConnected = intakeMaster.isConnected()

        if (isConnected) {
            intakeMaster.canSparkMax.restoreFactoryDefaults()

            intakeMaster.outputInverted = true
            intakeMaster.smartCurrentLimit = IntakeConstants.kCurrentLimit
        } else {
            println("Did not initialize Intake")
        }
    }

    private class PeriodicIO {
        var current: SIUnit<Ampere> = 0.amps
        var voltage: SIUnit<Volt> = 0.volts
        var position: SIUnit<NativeUnit> = 0.nativeUnits

        var feedforward: SIUnit<Volt> = 0.volts
        var desiredOutput: Output =
            Output.Nothing
    }

    override fun periodic() {
        val now = Timer.getFPGATimestamp()
        if (isConnected) {
            periodicIO.voltage = intakeMaster.voltageOutput
            periodicIO.current = intakeMaster.drawnCurrent
            periodicIO.position = intakeMaster.encoder.position

            when (val desiredOutput = periodicIO.desiredOutput) {
                is Output.Nothing ->
                    intakeMaster.setNeutral()
                is Output.Percent -> intakeMaster.setDutyCycle(desiredOutput.percent, periodicIO.feedforward)
            }
        }
        if (Timer.getFPGATimestamp() - now > 0.02) {
            println("Intake periodic() loop overrun.")
        }
    }

    sealed class Output {
        object Nothing : Output()
        class Percent(val percent: Double) : Output()
    }

    override fun setNeutral() {
        periodicIO.desiredOutput =
            Output.Nothing
        periodicIO.feedforward = 0.volts
    }

    fun setPercent(percent: Double) {
        periodicIO.desiredOutput =
            Output.Percent(percent)
        periodicIO.feedforward = 0.volts
    }

    fun extendPiston(extend: Boolean) {
        intakePiston.set(extend)
    }

    fun resetPosition(position: SIUnit<NativeUnit>) {
        intakeMaster.encoder.resetPosition(position)
    }
}
