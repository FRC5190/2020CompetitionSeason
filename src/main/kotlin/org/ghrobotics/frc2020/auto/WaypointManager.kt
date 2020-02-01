package org.ghrobotics.frc2020.auto

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.feet

/**
 * Handles all waypoints on the field.
 */
@Suppress("MemberVisibilityCanBePrivate")
object WaypointManager {
    /**
     * High goal on the opposite side of the field (scoring side).
     */
    val kGoal = Pose2d(54.feet, 7.254.feet, 0.degrees)

    /**
     * Good scoring location for the initial balls in auto.
     */
    val kOpenScoringLocation = Pose2d(38.21.feet, 4.75.feet, Rotation2d.fromDegrees(-132.0))

    val kDefaultStartingLocation: Pose2d = TODO()
}