/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.vision

import edu.wpi.first.wpilibj.geometry.Pose2d
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.vision.TargetTracker

/**
 * Keeps track of the field-relative goal pose for turret
 * rotation and latency compensation.
 */
object GoalTracker : TargetTracker(
    TargetTrackerConstants(
        VisionConstants.kMaxTargetTrackingLifetime,
        VisionConstants.kTargetTrackingDistanceErrorTolerance,
        VisionConstants.kMedianWindowSize
    )
) {
    /**
     * Returns whether any targets are currently being tracked.
     *
     * @return Whether any targets are currently being tracked.
     */
    val doTargetsExist: Boolean
        get() = super.targets.size > 0

    /**
     * Adds a sample to the Target Tracker.
     *
     * @param timestamp The time of capture.
     * @param pose The field-relative pose of the target.
     */
    fun addSample(timestamp: SIUnit<Second>, pose: Pose2d) {
        super.addSamples(timestamp, listOf(pose))
    }

    /**
     * Returns the closest target to the current robot pose.
     *
     * @param robotPose The current robot pose.
     *
     * @return THe closest target to the current robot pose.
     */
    fun getBestTarget(robotPose: Pose2d): TrackedTarget? {
        return targets.minBy { it.averagePose.translation.getDistance(robotPose.translation) }
    }
}
