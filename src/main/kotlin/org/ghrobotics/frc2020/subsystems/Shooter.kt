/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.ghrobotics.frc2020.ShooterConstants
import org.ghrobotics.frc2020.commands.TestShooterCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.AngularVelocity
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.subsystems.SensorlessCompatibleSubsystem

/**
 * Represents the shooter subsystem on the robot.
 */
object Shooter : FalconSubsystem(), SensorlessCompatibleSubsystem {

    // Create the master motor.
    private val masterMotor = FalconMAX(
        id = ShooterConstants.kMasterId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = ShooterConstants.kNativeUnitModel
    )

    // Feedforward for the shooter.
    private val feedforward = SimpleMotorFeedforward(
        ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA
    )

    // PeriodicIO.
    private val periodicIO = PeriodicIO()

    // Getter for speed
    val velocity get() = periodicIO.velocity

    // Initialize and configure motors.
    init {
        val slaveMotor = FalconMAX(
            id = ShooterConstants.kSlaveId,
            type = CANSparkMaxLowLevel.MotorType.kBrushless,
            model = ShooterConstants.kNativeUnitModel
        )
        slaveMotor.follow(masterMotor)

        masterMotor.canSparkMax.openLoopRampRate = 0.5
        masterMotor.canSparkMax.closedLoopRampRate = 0.5

        enableClosedLoopControl()
        defaultCommand = InstantCommand(Runnable { setPercent(0.0) }, this).perpetually()
    }

    /**
     * Returns the shooter subsystem check command.
     */
    override fun checkSubsystem(): Command {
        return TestShooterCommand().withTimeout(3.0)
    }

    /**
     * Enables closed loop control.
     */
    override fun enableClosedLoopControl() {
        masterMotor.controller.p = ShooterConstants.kP
    }

    /**
     * Disables closed loop control.
     */
    override fun disableClosedLoopControl() {
        masterMotor.controller.p = 0.0
    }

    /**
     * Sets the duty cycle of the shooter motor.
     *
     * @param percent The desired duty cycle.
     */
    fun setPercent(percent: Double) {
        periodicIO.desiredOutput = Output.Percent(percent)
        periodicIO.feedforward = 0.volts
    }

    /**
     * Sets the speed of the shooter.
     *
     * @param speed The desired speed.
     */
    fun setSpeed(speed: SIUnit<AngularVelocity>) {
        periodicIO.desiredOutput = Output.Velocity(speed)
        periodicIO.feedforward = SIUnit(feedforward.calculate(speed.value))
    }

    /**
     * Idles the shooter motor.
     */
    override fun setNeutral() {
        periodicIO.desiredOutput = Output.Nothing
        periodicIO.feedforward = 0.volts
    }

    override fun periodic() {
        periodicIO.velocity = masterMotor.encoder.velocity
        periodicIO.voltage = masterMotor.voltageOutput
        periodicIO.current = masterMotor.drawnCurrent

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> masterMotor.setNeutral()
            is Output.Percent -> masterMotor.setDutyCycle(desiredOutput.percent, periodicIO.feedforward)
            is Output.Velocity -> masterMotor.setVelocity(desiredOutput.velocity, periodicIO.feedforward)
        }
    }

    private class PeriodicIO {
        var velocity: SIUnit<AngularVelocity> = 0.radians / 1.seconds
        var voltage: SIUnit<Volt> = 0.volts
        var current: SIUnit<Ampere> = 0.amps

        var feedforward: SIUnit<Volt> = 0.volts
        var desiredOutput: Output = Output.Nothing
    }

    private sealed class Output {
        object Nothing : Output()
        class Velocity(val velocity: SIUnit<AngularVelocity>) : Output()
        class Percent(val percent: Double) : Output()
    }
}
