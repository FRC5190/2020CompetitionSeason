@file:Suppress("unused")

package org.ghrobotics.frc2020.auto

import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inches

/**
 * Handles all waypoints on the field.
 */
object WaypointManager {

    // Robot Constants
    private val kRobotLength = 31.inches
    private val kRobotWidth = 29.inches
    private val kBumperThickness = 3.inches

    // Field Constants
    private val kMaxFieldX = 54.feet
    private val kMaxFieldY = 27.feet

    private val kInitiationLineX = kMaxFieldX - 10.feet
    private val kGoalY = 7.254.feet

    // Waypoints
    // Steal Autos
    val kStealStart = Pose2d(
        x = kInitiationLineX,
        y = kMaxFieldY - kBumperThickness - kRobotWidth / 2.0,
        angle = Rotation2d.fromDegrees(180.0)
    )
    val kOpponentTrenchBalls = Pose2d(
        x = 33.31.feet, y = 24.38.feet, angle = Rotation2d.fromDegrees(156.0)
    )
    val kScoreAfterSteal = Pose2d(
        x = 41.78.feet, y = 03.26.feet, angle = Rotation2d.fromDegrees(134.0)
    )
    val kShortTrenchAfterSteal = Pose2d(
        x = 27.46.feet, y = 02.18.feet, angle = Rotation2d.fromDegrees(165.0)
    )
    val kLongTrenchAfterSteal = Pose2d(
        x = 22.56.feet, y = 02.13.feet, angle = Rotation2d.fromDegrees(173.0)
    )

    val kOpenScoringLocation = Pose2d(38.21.feet, 4.75.feet, Rotation2d.fromDegrees(-132.0))
}