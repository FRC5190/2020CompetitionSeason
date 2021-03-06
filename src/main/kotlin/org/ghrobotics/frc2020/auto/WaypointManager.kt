/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

@file:Suppress("unused")

package org.ghrobotics.frc2020.auto

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
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

    // Goal
    val kGoalLocation = Pose2d(
            x = kMaxFieldX, y = kGoalY, angle = Rotation2d()
    )

    // Waypoints
    val kTrenchStart = Pose2d(
            x = kInitiationLineX - kRobotLength / 2.0,
            y = 2.18.feet,
            angle = Rotation2d.fromDegrees(180.0)
    )

    val kRendezvousPickup: Pose2d = Pose2d(
            x = 31.23.feet, y = 12.12.feet, angle = Rotation2d.fromDegrees(113.0)
    ) // + Transform2d((-2).feet, -6.inches, Rotation2d.fromDegrees(0.0))

    val kTrenchPickup = Pose2d(
            x = 22.36.feet + 69.inches, y = 02.18.feet + 5.inches, angle = Rotation2d.fromDegrees(180.0)
    )

    val kTrenchScore = Pose2d(
            x = 40.65.feet, y = 05.03.feet, angle = Rotation2d.fromDegrees(200.0)
    )

    // Distances
    val kTrenchScoringDistance =
            SIUnit<Meter>(kTrenchScore.translation.getDistance(kGoalLocation.translation))
}
