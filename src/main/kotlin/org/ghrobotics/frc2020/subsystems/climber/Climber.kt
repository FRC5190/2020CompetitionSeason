/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.climber

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2020.ClimberConstants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.utils.isConnected

/**
 * Represents the climber assembly and the climber winch
 * on the robot.
 */
object Climber : FalconSubsystem() {

    // Winch motors.
    private val winchMasterMotor = FalconMAX(
        id = ClimberConstants.kWinchMasterId,
        type = CANSparkMaxLowLevel.MotorType.kBrushed,
        model = DefaultNativeUnitModel
    )
    private val winchSlaveMotor = FalconMAX(
        id = ClimberConstants.kWinchSlaveId,
        type = CANSparkMaxLowLevel.MotorType.kBrushed,
        model = DefaultNativeUnitModel
    )

    // Extension piston and brake to lock winch.
    private val extensionPiston = Solenoid(ClimberConstants.kPCMId, ClimberConstants.kExtensionPistonId)
    private val winchBrake = Solenoid(ClimberConstants.kPCMId, ClimberConstants.kWinchBrakeId)

    private val isConnected: Boolean

    private val periodicIO = PeriodicIO()

    init {
        isConnected = winchMasterMotor.isConnected() && winchSlaveMotor.isConnected()
        if (isConnected) {
            winchMasterMotor.canSparkMax.restoreFactoryDefaults()
            winchSlaveMotor.canSparkMax.restoreFactoryDefaults()

            // Slaves to follow master.
            winchSlaveMotor.follow(winchMasterMotor)

            // Set default command.
            defaultCommand = ManualClimberCommand { 0.0 }
        }
    }

    override fun periodic() {
        if (isConnected) {
            periodicIO.voltage = winchMasterMotor.voltageOutput
            periodicIO.current = winchMasterMotor.drawnCurrent

            when (val desiredOutput = periodicIO.desiredOutput) {
                is Output.Nothing -> {
                    winchMasterMotor.setNeutral()
                }
                is Output.Percent ->
                    winchMasterMotor.setDutyCycle(desiredOutput.percent, periodicIO.feedforward)
            }
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
        winchBrake.set(braked)
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
