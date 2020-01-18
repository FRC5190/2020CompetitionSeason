/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.turret

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.ghrobotics.frc2020.TurretConstants
import org.ghrobotics.frc2020.planners.TurretPlanner
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.AngularVelocity
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.subsystems.SensorlessCompatibleSubsystem

/**
 * Represents the turret on the robot.
 */
object Turret : FalconSubsystem(), SensorlessCompatibleSubsystem {

    // Hardware
    val master = FalconMAX(
        id = TurretConstants.kTurretId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = TurretConstants.kNativeUnitModel
    )

    private val hallEffectSensor = DigitalInput(TurretConstants.kHallEffectSensorId)

    // PeriodicIO.
    private val periodicIO = PeriodicIO()

    // Getters
    val angle get() = periodicIO.position
    val speed get() = periodicIO.velocity
    val current get() = periodicIO.current
    val hallEffectEngaged get() = periodicIO.hallEffect

    // Status (zeroing or ready)
    var status = Status.ZEROING
        private set

    init {
        master.outputInverted = true

        master.useMotionProfileForPosition = true
        master.motionProfileCruiseVelocity = TurretConstants.kMaxVelocity
        master.motionProfileAcceleration = TurretConstants.kMaxAcceleration

        master.brakeMode = true

        master.canSparkMax.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kReverse,
            TurretConstants.kNativeUnitModel.toNativeUnitPosition(TurretConstants.kAcceptableRange.start)
                .value.toFloat()
        )
        master.canSparkMax.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kForward,
            TurretConstants.kNativeUnitModel.toNativeUnitPosition(TurretConstants.kAcceptableRange.endInclusive)
                .value.toFloat()
        )

        defaultCommand = InstantCommand(Runnable {
            setPercent(
                0.0
            )
        }, this).perpetually()
        enableClosedLoopControl()
    }

    /**
     * Sets the zero of the turret at the current position.
     */
    fun zero() {
        master.encoder.resetPosition(0.radians)
        status =
            Status.READY
    }

    /**
     * Sets the duty cycle of the turret motor.
     *
     * @param percent The desired duty cycle.
     */
    fun setPercent(percent: Double) {
        periodicIO.desiredOutput =
            Output.Percent(percent)
        periodicIO.feedforward = 0.volts
    }

    /**
     * Idles the turret motor.
     */
    override fun setNeutral() {
        setPercent(0.0)
    }

    /**
     * Sets the angle of the turret
     *
     * @param angle The angle of the turret.
     */
    fun setAngle(angle: SIUnit<Radian>) {
        periodicIO.desiredOutput =
            Output.Position(
                TurretPlanner.constrainToAcceptableRange(angle)
            )
        periodicIO.feedforward = TurretConstants.kS
    }

    override fun enableClosedLoopControl() {
        master.controller.p = TurretConstants.kP
        master.controller.ff = TurretConstants.kF
    }

    override fun disableClosedLoopControl() {
        master.controller.p = 0.0
        master.controller.ff = 0.0
    }

    override fun periodic() {
        // Read sensor values.
        periodicIO.position = master.encoder.position
        periodicIO.velocity = master.encoder.velocity
        periodicIO.voltage = master.voltageOutput
        periodicIO.current = master.drawnCurrent

        periodicIO.hallEffect = hallEffectSensor.get()

        // Write motor outputs.
        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> master.setNeutral()
            is Output.Percent -> {
                master.setDutyCycle(desiredOutput.percent, periodicIO.feedforward)
            }
            is Output.Position -> {
                master.setPosition(desiredOutput.angle, periodicIO.feedforward)
            }
        }
    }

    private class PeriodicIO {
        var position: SIUnit<Radian> = 0.radians
        var velocity: SIUnit<AngularVelocity> = 0.radians / 1.seconds
        var voltage: SIUnit<Volt> = 0.volts
        var current: SIUnit<Ampere> = 0.amps
        var hallEffect: Boolean = false

        var feedforward: SIUnit<Volt> = 0.volts
        var desiredOutput: Output =
            Output.Nothing
    }

    private sealed class Output {
        object Nothing : Output()
        class Percent(val percent: Double) : Output()
        class Position(val angle: SIUnit<Radian>) : Output()
    }

    enum class Status {
        ZEROING, READY
    }
}
