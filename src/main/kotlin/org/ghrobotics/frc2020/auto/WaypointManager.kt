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
import org.ghrobotics.lib.mathematics.twodim.geometry.Transform2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
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

    val kControlPanelRegion = Rectangle2d(
        Translation2d(21.5.feet, 0.feet), Translation2d(25.feet, 4.5.feet)
    )

    // Waypoints
    // Steal Autos
    val kStealStart = Pose2d(
        x = kInitiationLineX,
        y = 24.82.feet,
        angle = Rotation2d.fromDegrees(180.0)
    )
    val kOpponentTrenchBalls = Pose2d(
        x = 33.31.feet, y = 25.38.feet, angle = Rotation2d.fromDegrees(156.0)
    )
    val kStealAutoIntermediate = Pose2d(
        x = 39.10.feet, y = 24.17.feet, angle = Rotation2d.fromDegrees(-153.0)
    )

    // Trench Autos
    val kTrenchStart = Pose2d(
        x = kInitiationLineX - kRobotLength / 2.0 - kBumperThickness,
        y = 2.18.feet,
        angle = Rotation2d()
    )

    val kShortPickupAfterTrench = Pose2d(
        x = 28.12.feet, y = 2.18.feet, angle = Rotation2d.fromDegrees(180.0)
    )
    val kLongPickupAfterTrench = Pose2d(
        x = 22.00.feet, y = 2.35.feet, angle = Rotation2d.fromDegrees(169.0)
    )

    // Scoring Locations
    val kGoodInnerGoalScoringLocation = Pose2d(
        x = 37.54.feet, y = 6.95.feet, angle = Rotation2d.fromDegrees(-90.0)
    )
    val kGoodTrenchScoringLocation = Pose2d(
        x = 35.98.feet, y = 5.91.feet, angle = Rotation2d.fromDegrees(215.0)
    )
    val kGoodAfterStealScoringLocation = Pose2d(
        x = 38.98.feet, y = 9.21.feet, angle = Rotation2d.fromDegrees(-90.0)
    )
}
