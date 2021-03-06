/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.climber

import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.frc2020.kPCMId
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX

/**
 * Represents the climber assembly and the climber winch
 * on the robot.
 */
object Climber : FalconSubsystem() {

    // Winch motors.
    private val winchMasterMotor = FalconSRX(
        id = ClimberConstants.kWinchMasterId,
        model = DefaultNativeUnitModel
    )
    private val winchSlaveMotor = FalconSRX(
        id = ClimberConstants.kWinchSlaveId,
        model = DefaultNativeUnitModel
    )

    // Extension piston and brake to lock winch.
    private val extensionPiston = Solenoid(kPCMId, ClimberConstants.kExtensionPistonId)
    private val winchBrake = Solenoid(kPCMId, ClimberConstants.kWinchBrakeId)

    // Connection status
    private var isConnected = true

    // PeriodicIO
    private val periodicIO = PeriodicIO()

    // Getters
    var isWinchLocked = false
        private set

    val current get() = periodicIO.current

    override fun lateInit() {
        if (isConnected) {
            // Slaves to follow master.
            winchSlaveMotor.follow(winchMasterMotor)

            // Set default command.
            defaultCommand = ClimberPercentCommand { 0.0 }

            setWinchBrake(false)
        } else {
            println("Did not initialize Climber.")
        }
    }

    override fun periodic() {
        val now = Timer.getFPGATimestamp()
        if (isConnected) {
            periodicIO.voltage = winchMasterMotor.voltageOutput
            periodicIO.current = winchMasterMotor.drawnCurrent

            when (val desiredOutput = periodicIO.desiredOutput) {
                is Output.Nothing -> winchMasterMotor.setNeutral()
                is Output.Percent -> winchMasterMotor.setDutyCycle(desiredOutput.percent, periodicIO.feedforward)
            }
        }
        if (Timer.getFPGATimestamp() - now > 0.02) {
            println("Climber periodic() loop overrun.")
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
     * Sets a percent on the winch.
     *
     * @param percent The percent to set.
     */
    fun setPercent(percent: Double) {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.Percent(percent)
    }

    /**
     * Extends or retracts the climber assembly.
     *
     * @param extend Whether to extend the climber assembly or not.
     */
    fun extend(extend: Boolean) {
        extensionPiston.set(extend)
    }

    /**
     * Sets the winch brake.
     *
     * @param braked Whether to enable the brake or not.
     */
    fun setWinchBrake(braked: Boolean) {
        isWinchLocked = braked
        winchBrake.set(!braked)
    }

    private class PeriodicIO {
        var current: SIUnit<Ampere> = 0.amps
        var voltage: SIUnit<Volt> = 0.volts

        var feedforward: SIUnit<Volt> = 0.volts
        var desiredOutput: Output = Output.Nothing
    }

    sealed class Output {
        object Nothing : Output()
        class Percent(val percent: Double) : Output()
    }
}
