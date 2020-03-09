/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.vision

import edu.wpi.first.wpilibj.MedianFilter
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Transform2d
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.seconds

/**
 * Manages the Limelight on the robot.
 */
object LimelightManager : FalconSubsystem() {

    // Constants
    private val kCameraHeight = 24.125.inches
    private val kGoalHeight = 89.5.inches
    private val kCameraAngle = 25.degrees

    private val kSwitchToZoomedPipelineThresold = (-6).degrees
    private val kTxThreshold = 8.degrees
    private val kPitchOffsetWhenZoomed = (-12.425).degrees

    private const val kDefaultPipelineId = 0
    private const val kZoomedPipelineId = 1

    private const val kPitchFilterDataPoints = 5

    // Shooter Limelight
    private val turretLimelight = Limelight()

    // Private states
    private val pitchFilter = MedianFilter(kPitchFilterDataPoints)
    private val yawFilter = MedianFilter(kPitchFilterDataPoints)

    /**
     * Returns whether the turret limelight is connected.
     */
    fun isTurretLimelightConnected() = turretLimelight.isConnected()

    /**
     * Returns whether the turret Limelight has a valid target.
     */
    fun doesTurretLimelightHaveValidTarget() = turretLimelight.hasValidTarget

    /**
     * Returns the distance to the target.
     */
    fun getDistanceToGoal(): SIUnit<Meter> {
        // Adjust pitch for zoom setting.
        val adjustedPitch = if (getSelectedPipeline() == kDefaultPipelineId) {
            turretLimelight.pitch
        } else {
            turretLimelight.pitch + kPitchOffsetWhenZoomed
        }

        // Calculate and return distance.
        return (kGoalHeight - kCameraHeight) / tan((adjustedPitch + kCameraAngle).value)
    }

    /**
     * Returns the angle to the target.
     */
    fun getAngleToGoal(): SIUnit<Radian> = turretLimelight.yaw

    /**
     * Returns the selected pipeline.
     */
    fun getSelectedPipeline(): Int = turretLimelight.selectedPipeline

    /**
     * Turns on the Limelight LED.
     */
    fun turnOnLED() = turretLimelight.turnOnLED()

    /**
     * Turns off the Limelight LED.
     */
    fun turnOffLED() = turretLimelight.turnOffLED()

    /**
     * Blinks the Limelight LED.
     */
    fun blinkLED() = turretLimelight.blinkLED()

    /**
     * Updates the Limelight.
     */
    override fun periodic() {
        turretLimelight.update()

        // Handle switching of pipelines.
        if (getSelectedPipeline() == kDefaultPipelineId &&
            pitchFilter.calculate(turretLimelight.pitch.value) < kSwitchToZoomedPipelineThresold.value &&
                yawFilter.calculate(turretLimelight.yaw.value).absoluteValue < kTxThreshold.value
        ) {
            // Zoom in if the zoomed pipeline can see the target.
            turretLimelight.setPipeline(kZoomedPipelineId)
        } else if (getSelectedPipeline() == kZoomedPipelineId && !doesTurretLimelightHaveValidTarget()) {
            // Zoom out if the zoomed pipeline cannot see the target.
            turretLimelight.setPipeline(kDefaultPipelineId)
            pitchFilter.reset()
            yawFilter.reset()
        }

        if (doesTurretLimelightHaveValidTarget()) {
            // Calculate image timestamp.
            val timestamp = Timer.getFPGATimestamp().seconds - turretLimelight.latency

            // Get camera-relative goal pose at timestamp.
            val cameraToGoal = Transform2d(
                getDistanceToGoal() * cos(getAngleToGoal().value),
                getDistanceToGoal() * sin(getAngleToGoal().value), Rotation2d()
            )

            // Get field-relative goal pose at timestamp.
            val fieldToGoal = Drivetrain.getPose(timestamp) +
                Turret.getRobotToTurret(timestamp) + VisionConstants.kTurretToCamera + cameraToGoal

            // Add goal pose to GoalTracker.
            GoalTracker.addSample(timestamp, Pose2d(fieldToGoal.translation, Rotation2d()))
        }
    }
}
