/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.vision

import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Transform2d
import kotlin.math.tan
import org.ghrobotics.frc2020.Robot
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.utils.toTransform
import org.ghrobotics.lib.wrappers.FalconTimedRobot

/**
 * Object that handles Vision Processing on the robot.
 */
object VisionProcessing : FalconSubsystem() {

    // The Green LED to track targets.
    private val led = DigitalOutput(VisionConstants.kLEDId)

    // The camera on the turret.
    private val camera = ChameleonCamera("USB Camera-B4.09.24.1")

    // PeriodicIO.
    private val periodicIO = PeriodicIO()
    private var lastDesiredOutput = true

    /**
     * Returns the angle to the best target.
     *
     * @return The angle to the best target.
     */
    val angle get() = camera.yaw

    /**
     * Returns whether the camera sees a valid target.
     */
    val isValid get() = camera.isValid

    /**
     * Returns the distance along the ground to the goal.
     *
     * @return The distance along the ground to the goal.
     */
    val distance: SIUnit<Meter>
        get() {
            val deltaHeight = VisionConstants.kGoalHeight - VisionConstants.kCameraHeight
            return deltaHeight / tan(camera.pitch.radians + VisionConstants.kCameraAngle.radians)
        }

    override fun periodic() {
        // Update camera.
        camera.update()

        // Add solvePnP pose to GoalTracker.
        val cameraToTarget: Transform2d? = camera.transform
        if (cameraToTarget != null) {
            val latency = camera.latency
            val timestamp = Timer.getFPGATimestamp().seconds - latency

            val turretToTarget = VisionConstants.kTurretToCamera + cameraToTarget
            val robotToTarget = Turret.getRobotToTurret(timestamp) + turretToTarget.toTransform()
            val fieldRelativeTarget = Drivetrain.getPose(timestamp) + robotToTarget.toTransform()

            GoalTracker.addSample(timestamp, fieldRelativeTarget)
        }

        // Update GoalTracker.
        GoalTracker.update()

        if (Robot.currentMode == FalconTimedRobot.Mode.DISABLED) {
            periodicIO.desiredLEDState = !camera.isConnected
        }
        if (periodicIO.desiredLEDState != lastDesiredOutput) {
            led.set(!periodicIO.desiredLEDState)
            lastDesiredOutput = periodicIO.desiredLEDState
        }
    }

    /**
     * Turns on the LEDs for vision tracking.
     */
    fun turnOnLEDs() {
        periodicIO.desiredLEDState = true
    }

    /**
     * Turns off the LEDs for vision tracking.
     */
    fun turnOffLEDs() {
        periodicIO.desiredLEDState = false
    }

    private class PeriodicIO {
        var desiredLEDState: Boolean = false
    }
}
