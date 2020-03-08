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

    // Waypoints
    val kStealStart = Pose2d(
        x = kInitiationLineX - kRobotLength / 2.0,
        y = 24.82.feet,
        angle = Rotation2d.fromDegrees(180.0)
    )
    val kTrenchStart = Pose2d(
        x = kInitiationLineX - kRobotLength / 2.0,
        y = 2.18.feet,
        angle = Rotation2d.fromDegrees(180.0)
    )

    val kGoalLocation = Pose2d(
        x = kMaxFieldX, y = kGoalY, angle = Rotation2d()
    )

    val kOpponentTrenchBalls = Pose2d(
        x = 34.00.feet, y = 24.82.feet, angle = Rotation2d.fromDegrees(181.0)
    )

    val kProtectedScoringLocation = Pose2d(
        x = 50.21.feet, y = 09.06.feet, angle = Rotation2d.fromDegrees(102.0)
    )

    val kTrenchScoringLocation = Pose2d(
        x = 42.16.feet, y = 07.42.feet, angle = Rotation2d.fromDegrees(-159.0)
    )

    val kTrenchPickup = Pose2d(
        x = 23.36.feet, y = 02.18.feet, angle = Rotation2d.fromDegrees(180.0)
    )

    val kInitLineScoringLocation = Pose2d(
        x = 42.54.feet, y = 11.20.feet, angle = Rotation2d.fromDegrees(132.0)
    )

    val kDoubleRendezvousPickup = Pose2d(
        x = 31.15.feet, y = 19.94.feet, angle = Rotation2d.fromDegrees(-157.0)
    )

    val kRendezvousPickupIntermediate = Pose2d(
        x = 39.03.feet, y = 14.28.feet, angle = Rotation2d.fromDegrees(-157.0)
    )

    val kSingleRendezvousPickup = Pose2d(
        x = 25.74.feet, y = 14.28.feet, angle = Rotation2d.fromDegrees(-157.0)
    )

    val kTrenchRendezvousPickup = Pose2d(
        x = 33.97.feet, y = 08.05.feet, angle = Rotation2d.fromDegrees(112.0)
    ) + Transform2d(-5.inches, 0.inches, Rotation2d.fromDegrees(0.0))

    val kTrenchPickupIntermediate = Pose2d(
        x = 40.65.feet, y = 03.82.feet, angle = Rotation2d.fromDegrees(158.0)
    )

    // Distances
    val kInitLineScoringDistance =
        SIUnit<Meter>(kInitLineScoringLocation.translation.getDistance(kGoalLocation.translation))

    val kProtectedScoringDistance =
        SIUnit<Meter>(kProtectedScoringLocation.translation.getDistance(kGoalLocation.translation))

    val kTrenchRedezvousScoringDistance =
        SIUnit<Meter>(kTrenchRendezvousPickup.translation.getDistance(kGoalLocation.translation))

    val kTrenchScoringDistance =
        SIUnit<Meter>(kTrenchScoringLocation.translation.getDistance(kGoalLocation.translation))
}
