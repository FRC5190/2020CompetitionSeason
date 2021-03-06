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
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Transform2d
import kotlin.math.atan2
import org.ghrobotics.frc2020.auto.WaypointManager
import org.ghrobotics.frc2020.isConnected
import org.ghrobotics.frc2020.planners.TurretPlanner
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.vision.GoalTracker
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.AngularVelocity
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.utils.InterpolatingTreeMapBuffer

/**
 * Represents the turret on the robot.
 */
object Turret : FalconSubsystem() {

    // Hardware
    private val master = FalconMAX(
            id = TurretConstants.kTurretId,
            type = CANSparkMaxLowLevel.MotorType.kBrushless,
            model = TurretConstants.kNativeUnitModel
    )

    private val hallEffectSensor = DigitalInput(TurretConstants.kHallEffectSensorId)

    // PeriodicIO.
    private val periodicIO = PeriodicIO()

    // Connection status
    private var isConnected = false

    // Buffer to store previous turret angles for latency compensation.
    private val buffer =
            InterpolatingTreeMapBuffer.createFromSI<Second, Radian>(1.seconds) { Timer.getFPGATimestamp().seconds }

    // Lambda that dictates the turret's default behavior.
    val defaultBehavior: () -> SIUnit<Radian> = {
        if (GoalTracker.isTrackingTargets) {
            GoalTracker.latestTurretAngleToFaceOuterGoal
        } else {
            -Drivetrain.getAngle()
        }
    }

    val innerGoalBehavior: () -> SIUnit<Radian> = {
        if (GoalTracker.isTrackingTargets) {
            GoalTracker.latestTurretAngleToFaceInnerGoal
        } else {
            -Drivetrain.getAngle()
        }
    }

    val autoBehavior: () -> SIUnit<Radian> = {
        if (GoalTracker.isTrackingTargets) {
            GoalTracker.latestTurretAngleToFaceInnerGoal
        } else {
            // Odometry will be accurate in auto, so we can rely on that.
            val pose = Drivetrain.getPose()
            val goal = WaypointManager.kGoalLocation
            atan2(goal.y - pose.y, goal.x - pose.x).radians - Drivetrain.getAngle()
        }
    }

    // Getters
    val speed get() = periodicIO.velocity
    val current get() = periodicIO.current
    val hallEffectEngaged get() = periodicIO.hallEffect

    /**
     * Returns the angle at the specified timestamp.
     *
     * @param timestamp The timestamp.
     * @return The angle at the specified timestamp.
     */
    fun getAngle(timestamp: SIUnit<Second> = Timer.getFPGATimestamp().seconds): SIUnit<Radian> {
        return buffer[timestamp] ?: {
            DriverStation.reportError("[Turret] Buffer was empty!", false)
            0.degrees
        }()
    }

    var distance: SIUnit<Meter> = 0.meters
        private set

    /**
     * Returns the turret position with the robot's center as the origin of
     * the coordinate frame at the specified timestamp.
     *
     * @param timestamp The timestamp.
     * @return The turret position with the robot's center as the origin of
     *         the coordinate frame at the specified timestamp.
     */
    fun getRobotToTurret(timestamp: SIUnit<Second> = Timer.getFPGATimestamp().seconds): Transform2d {
        return Transform2d(TurretConstants.kTurretRelativeToRobotCenter, getAngle(timestamp).toRotation2d())
    }

    // Status (zeroing or ready)
    var status = Status.NOT_ZEROED
        private set

    override fun lateInit() {
        isConnected = master.isConnected()

        // Check if the Spark is on the bus.
        if (isConnected) {
            master.canSparkMax.restoreFactoryDefaults()

            master.outputInverted = false

            master.useMotionProfileForPosition = true
            master.motionProfileCruiseVelocity = TurretConstants.kMaxVelocity
            master.motionProfileAcceleration = TurretConstants.kMaxAcceleration

            master.brakeMode = true

            master.canSparkMax.setSoftLimit(
                    CANSparkMax.SoftLimitDirection.kReverse,
                    TurretConstants.kNativeUnitModel.toNativeUnitPosition(
                            TurretConstants.kAcceptableRange.start
                    )
                            .value.toFloat()
            )
            master.canSparkMax.setSoftLimit(
                    CANSparkMax.SoftLimitDirection.kForward,
                    TurretConstants.kNativeUnitModel.toNativeUnitPosition(
                            TurretConstants.kAcceptableRange.endInclusive
                    ).value.toFloat()
            )

            master.controller.p = TurretConstants.kP
            master.controller.ff = TurretConstants.kF
        } else {
            println("Did not initialize Turret")
        }

        defaultCommand = TurretPositionCommand(defaultBehavior)
    }

    /**
     * Sets the zero of the turret at the current position.
     */
    fun zero() {
        periodicIO.resetPosition = true
        periodicIO.resetTo = TurretConstants.kZeroLocation
        setStatus(Status.READY)
    }

    /**
     * Sets the status of the turret.
     */
    fun setStatus(status: Status) {
        this.status = status
    }

    /**
     * Jogs the zero of the turret by a certain amount.
     *
     * @param jogAmount The amount to jog the zero by.
     */
    fun jogZero(jogAmount: SIUnit<Radian>) {
        periodicIO.resetPosition = true
        periodicIO.resetTo = periodicIO.position - jogAmount
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
                Output.Position(TurretPlanner.getOptimizedAngle(angle, periodicIO.position))
        periodicIO.feedforward = TurretConstants.kS
    }

    fun setBrakeMode(brakeMode: Boolean) {
        master.brakeMode = brakeMode
    }

    override fun periodic() {
        val now = Timer.getFPGATimestamp()
        if (isConnected) {
            // Read sensor values.
            periodicIO.position = master.encoder.position
            periodicIO.velocity = master.encoder.velocity
            periodicIO.voltage = master.voltageOutput
            periodicIO.current = master.drawnCurrent

            buffer[Timer.getFPGATimestamp().seconds] = periodicIO.position

            periodicIO.hallEffect = !hallEffectSensor.get()

            if (periodicIO.resetPosition) {
                periodicIO.resetPosition = false
                master.encoder.resetPosition(periodicIO.resetTo)
            }

            if (status == Status.READY) {
                // Write motor outputs.
                when (val desiredOutput = periodicIO.desiredOutput) {
                    is Output.Nothing -> master.setNeutral()
                    is Output.Percent -> master.setDutyCycle(desiredOutput.percent, periodicIO.feedforward)
                    is Output.Position -> master.setPosition(desiredOutput.angle, periodicIO.feedforward)
                }
            } else {
                master.setNeutral()
            }
        }
        if (Timer.getFPGATimestamp() - now > 0.02) {
            println("Turret periodic() loop overrun.")
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

        var resetPosition: Boolean = false
        var resetTo: SIUnit<Radian> = 0.radians
    }

    private sealed class Output {
        object Nothing : Output()
        class Percent(val percent: Double) : Output()
        class Position(val angle: SIUnit<Radian>) : Output()
    }

    enum class Status {
        NOT_ZEROED, ZEROING, READY
    }
}
