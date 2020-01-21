package org.ghrobotics.frc2020.vision

import edu.wpi.first.wpilibj.geometry.Pose2d
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
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
}