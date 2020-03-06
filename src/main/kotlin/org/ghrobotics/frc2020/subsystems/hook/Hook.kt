/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.hook

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.utils.isConnected

/**
 * Represents the hook which can slide across the climbing bar.
 */
object Hook : FalconSubsystem() {

    // Motor
    private val masterMotor = FalconMAX(
        id = HookConstants.kHookId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = DefaultNativeUnitModel
    )

    private val periodicIO = PeriodicIO()

    // Connection Status
    private var isConnected = false

    override fun lateInit() {
        isConnected = masterMotor.isConnected()
        if (isConnected) {
            masterMotor.canSparkMax.restoreFactoryDefaults()
            masterMotor.brakeMode = true
        } else {
            println("Did not initialize Hook")
        }
        defaultCommand = HookPercentCommand { 0.0 }
    }

    override fun periodic() {
        val now = Timer.getFPGATimestamp()
        periodicIO.voltage = masterMotor.voltageOutput
        periodicIO.current = masterMotor.drawnCurrent

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing ->
                masterMotor.setNeutral()
            is Output.Percent ->
                masterMotor.setDutyCycle(desiredOutput.percent, periodicIO.feedforward)
        }
        if (Timer.getFPGATimestamp() - now > 0.02) {
            println("Hook periodic() loop overrun.")
        }
    }

    /**
     * Zeros the motor output.
     */
    override fun setNeutral() {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.Nothing
    }

    /**
     * Commands a percent output to the motor.
     *
     * @param percent The percent output to command.
     */
    fun setPercent(percent: Double) {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.Percent(percent)
    }

    private class PeriodicIO {
        var current: SIUnit<Ampere> = 0.amps
        var voltage: SIUnit<Volt> = 0.volts
        var position: SIUnit<Meter> = 0.meters

        var feedforward: SIUnit<Volt> = 0.volts
        var desiredOutput: Output = Output.Nothing
    }

    private sealed class Output {
        object Nothing : Output()
        class Percent(val percent: Double) : Output()
    }
}
