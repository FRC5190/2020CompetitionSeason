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
import edu.wpi.first.wpilibj.geometry.Transform2d
import kotlin.math.tan
import org.ghrobotics.frc2020.TurretConstants
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.frc2020.subsystems.Drivetrain
import org.ghrobotics.frc2020.subsystems.Turret
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.seconds

/**
 * Object that handles Vision Processing on the robot.
 */
object VisionProcessing : FalconSubsystem() {
    // The Green LED to track targets.
    private val led = DigitalOutput(VisionConstants.kLEDId)

    // The camera on the turret.
    private val camera = ChameleonCamera("USB Camera-B4.09.24.1")

    /**
     * Returns the angle to the best target.
     *
     * @return The angle to the best target.
     */
    val angle get() = camera.yaw

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

    /**
     * Run periodically to update the target tracker with the latest data.
     */
    override fun periodic() {
        val transformToGoalFromCamera = camera.transform ?: return
        val timestamp = Timer.getFPGATimestamp().seconds - camera.latency

        val turretToGoal = VisionConstants.kCameraRelativeToTurretCenter + transformToGoalFromCamera
        val turretRelativeToRobot = Pose2d(TurretConstants.kTurretRelativeToRobotCenter, Turret.angle.toRotation2d())

        val goalRelativeToRobot = turretRelativeToRobot + turretToGoal.toTransform()
        val robotPose = Drivetrain.getPose(timestamp)

        GoalTracker.addSample(timestamp, robotPose + goalRelativeToRobot.toTransform())
    }

    /**
     * Turns on the LEDs for vision tracking.
     */
    fun turnOnLEDs() {
        led.set(false)
    }

    /**
     * Turns off the LEDs for vision tracking.
     */
    fun turnOffLEDs() {
        led.set(true)
    }

    private fun Pose2d.toTransform(): Transform2d {
        return Transform2d(translation, rotation)
    }
}
