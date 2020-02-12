/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryConfig
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds

/**
 * Stores all paths / trajectories for auto.
 */
object TrajectoryManager {

    // Constraints
    private val kMaxVelocity = 12.feet / 1.seconds
    private val kMaxAcceleration = 8.feet / 1.seconds / 1.seconds
    private val kMaxCentripetalAcceleration = 8.feet / 1.seconds / 1.seconds
    private val kMaxVoltage = 10.volts

    private val kCentripetalAccelerationConstraint =
        CentripetalAccelerationConstraint(kMaxCentripetalAcceleration.value)

    private val kVoltageConstraint =
        DifferentialDriveVoltageConstraint(Drivetrain.getFeedforward(), Drivetrain.kinematics, kMaxVoltage.value)

    // Config
    private val kFwdConfig = FalconTrajectoryConfig(kMaxVelocity, kMaxAcceleration)
        .setKinematics(Drivetrain.kinematics)
        .addConstraint(kCentripetalAccelerationConstraint)
        .addConstraint(kVoltageConstraint)

    private val kRevConfig = FalconTrajectoryConfig(kMaxVelocity, kMaxAcceleration)
        .setKinematics(Drivetrain.kinematics)
        .addConstraint(kCentripetalAccelerationConstraint)
        .addConstraint(kVoltageConstraint)
        .setReversed(true)

    // Trajectories
    // Steal Autos
    val stealStartToOpponentTrenchBalls: Trajectory =
        generate(WaypointManager.kStealStart, WaypointManager.kOpponentTrenchBalls, kFwdConfig)

    val opponentTrenchBallsToStealScore: Trajectory =
        TrajectoryGenerator.generateTrajectory(
            WaypointManager.kOpponentTrenchBalls, listOf(Translation2d(x = 38.69.feet, y = 13.18.feet)),
            WaypointManager.kScoreAfterSteal, kRevConfig
        )

    val stealScoreToShortPickup: Trajectory =
        generate(WaypointManager.kScoreAfterSteal, WaypointManager.kShortPickupAfterSteal, kFwdConfig)

    val stealScoreToLongPickup: Trajectory =
        TrajectoryGenerator.generateTrajectory(
            WaypointManager.kScoreAfterSteal, listOf(Translation2d(x = 30.63.feet, y = 02.71.feet)),
            WaypointManager.kLongPickupAfterSteal, kFwdConfig
        )

    val longPickupToShortPickup: Trajectory =
        generate(WaypointManager.kLongPickupAfterSteal, WaypointManager.kShortPickupAfterSteal, kRevConfig)

    // Trench Autos
    val trenchStartToShortPickup: Trajectory =
        generate(WaypointManager.kTrenchStart, WaypointManager.kShortPickupAfterTrench, kFwdConfig)

    val trenchStartToLongPickup: Trajectory =
        generate(WaypointManager.kTrenchStart, WaypointManager.kLongPickupAfterTrench, kFwdConfig)

    /**
     * Generates a trajectory from a start and end waypoint.
     */
    private fun generate(start: Pose2d, end: Pose2d, config: TrajectoryConfig): Trajectory =
        TrajectoryGenerator.generateTrajectory(
            start, listOf(), end, config
        )
}
