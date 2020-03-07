/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.vision

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.turret.TurretConstants
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.vision.TargetTracker

/**
 * Uses the FalconLibrary TargetTracker class to track the 2020 high-goal
 * vision target.
 */
object GoalTracker : TargetTracker(
    TargetTrackerConstants(
        VisionConstants.kMaxTargetTrackingLifetime,
        VisionConstants.kTargetTrackingDistanceErrorTolerance,
        VisionConstants.kMedianWindowSize
    )
) {
    /**
     * Returns the number of targets being tracked.
     */
    val numberOfTargets: Int
        get() = targets.size

    /**
     * Returns whether the GoalTracker is tracking any targets or not.
     */
    val isTrackingTargets: Boolean
        get() = numberOfTargets > 0

    /**
     * Latest turret to goal pose.
     */
    var latestTurretToGoal = Pose2d()
        private set

    /**
     * Latest turret to goal distance.
     */
    var latestTurretToGoalDistance: SIUnit<Meter> = 0.inches
        private set

    /**
     * Returns the closest target to the given field-relative pose.
     *
     * @param robotPose The field-relative robot pose.
     * @return The closest target to the given field-relative pose or null
     *         if no target exists.
     */
    fun getClosestTarget(robotPose: Pose2d): TrackedTarget? {
        return targets.minBy { it.averagePose.translation.getDistance(robotPose.translation) }
    }

    /**
     * Adds a sample to the GoalTracker.
     *
     * @param timestamp The time of capture.
     * @param sample The field-relative pose of the target.
     */
    fun addSample(timestamp: SIUnit<Second>, sample: Pose2d) {
        super.addSamples(timestamp, listOf(sample))
    }

    fun periodic() {
        val now = Timer.getFPGATimestamp()
        super.update()

        // Get field-relative turret pose.
        val fieldToTurret = Drivetrain.getPose() +
            Transform2d(TurretConstants.kTurretRelativeToRobotCenter, Rotation2d())

        // Get goal pose.
        val fieldToGoal = getClosestTarget(fieldToTurret)

        if (fieldToGoal != null) {
            // Get goal in turret coordinates.
            latestTurretToGoal = fieldToGoal.averagePose.relativeTo(fieldToTurret)

            // Calculate distance
            latestTurretToGoalDistance = SIUnit(latestTurretToGoal.translation.norm)
        }
        if (Timer.getFPGATimestamp() - now > 0.02) {
            println("GoalTracker periodic() overrun")
        }
    }
}