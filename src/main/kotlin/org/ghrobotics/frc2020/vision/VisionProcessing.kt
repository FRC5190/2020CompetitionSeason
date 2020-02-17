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
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import kotlin.math.tan
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.utils.toTransform

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
     * Returns whether the camera is connected.
     */
    val isConnected get() = camera.isConnected

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

    var estimatedRobotPose: Pose2d = Pose2d()
        private set

    override fun periodic() {
        // Update camera.
        camera.update()

        // Substitute (albeit very accurate) for solvePnP until solvePnP is fixed.
        val cameraToTarget = Transform2d(Translation2d(distance * angle.cos, distance * angle.sin), Rotation2d())

        // Add solvePnP pose to GoalTracker.
        if (cameraToTarget != Transform2d() && camera.isValid) {
            val latency = camera.latency
            val timestamp = Timer.getFPGATimestamp().seconds - latency

            val turretToTarget = VisionConstants.kTurretToCamera + cameraToTarget
            val robotToTarget = Turret.getRobotToTurret(timestamp) + turretToTarget.toTransform()
            val fieldRelativeTarget = Drivetrain.getPose(timestamp) + robotToTarget.toTransform()

            GoalTracker.addSample(timestamp, fieldRelativeTarget)
            estimatedRobotPose =
                GoalLocalizer.calculateRobotPose(timestamp, cameraToTarget, Drivetrain.getPose().rotation)
        }

        // Update GoalTracker.
        GoalTracker.update()

        led.set(!periodicIO.desiredLEDState)
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
