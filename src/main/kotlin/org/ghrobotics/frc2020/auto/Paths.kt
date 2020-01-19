/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryConfig
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inMeters
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds

/**
 * Stores all paths / trajectories for auto.
 */
object Paths {
    // The trajectory config
    private val config: TrajectoryConfig = FalconTrajectoryConfig(
        10.feet / 1.seconds,
        5.feet / 1.seconds / 1.seconds
    ).addConstraint(CentripetalAccelerationConstraint(5.feet.inMeters()))
        .setReversed(true)

    val frontOfGoalToTrench: Trajectory = TrajectoryGenerator.generateTrajectory(
        Pose2d(43.026.feet, 7.254.feet, Rotation2d()),
        listOf(Translation2d(36.049.feet, 3.613.feet), Translation2d(27.937.feet, 2.122.feet)),
        Pose2d(25.818.feet, 4.586.feet, Rotation2d.fromDegrees(-62.0)),
        config
    )
}
