/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

@file:Suppress("unused")

package org.ghrobotics.frc2020.auto

import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.derived.degrees
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
    private val kMinFieldX = 0.feet
    private val kMaxFieldX = 54.feet
    private val kMinFieldY = 0.feet
    private val kMaxFieldY = 27.feet

    private val kInitiationLineX = kMaxFieldX - 10.feet
    private val kGoalY = 7.254.feet

    // Regions
    val kTrenchRegion = Rectangle2d(
        Translation2d(18.feet, 0.feet), Translation2d(36.feet, 4.5.feet)
    )

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
    val kShortPickupAfterSteal = Pose2d(
        x = 27.46.feet, y = 02.18.feet, angle = Rotation2d.fromDegrees(165.0)
    )
    val kLongPickupAfterSteal = Pose2d(
        x = 22.56.feet, y = 02.13.feet, angle = Rotation2d.fromDegrees(173.0)
    )

    // Trench Autos
    val kTrenchStart = Pose2d(
        x = kInitiationLineX,
        y = 2.18.feet,
        angle = Rotation2d.fromDegrees(180.0)
    )
    val kShortPickupAfterTrench = Pose2d(
        x = 28.12.feet, y = 2.18.feet, angle = Rotation2d.fromDegrees(180.0)
    )
    val kLongPickupAfterTrench = Pose2d(
        x = 22.59.feet, y = 2.27.feet, angle = Rotation2d.fromDegrees(175.0)
    )

    // Rendezvous Autos
    val kNearRendezvous = Pose2d(
        x = 33.82.feet, y = 8.61.feet, angle = Rotation2d.fromDegrees(135.0)
    )

    val kOpenScoringLocation = Pose2d(38.21.feet, 4.75.feet, Rotation2d.fromDegrees(-132.0))
}
