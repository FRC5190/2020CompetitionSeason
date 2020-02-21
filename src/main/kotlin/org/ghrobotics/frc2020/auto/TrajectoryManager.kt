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
    private val kMaxPickupVelocity = 7.feet / 1.seconds

    private val kMaxAcceleration = 11.feet / 1.seconds / 1.seconds
    private val kMaxPickupAcceleration = 10.feet / 1.seconds / 1.seconds

    private val kMaxCentripetalAcceleration = 12.feet / 1.seconds / 1.seconds
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

    private val kFwdPickupConfig = FalconTrajectoryConfig(kMaxPickupVelocity, kMaxPickupAcceleration)
        .setKinematics(Drivetrain.kinematics)
        .addConstraint(kCentripetalAccelerationConstraint)
        .addConstraint(kVoltageConstraint)

    private val kRevPickupConfig = FalconTrajectoryConfig(kMaxPickupVelocity, kMaxPickupAcceleration)
        .setKinematics(Drivetrain.kinematics)
        .addConstraint(kCentripetalAccelerationConstraint)
        .addConstraint(kVoltageConstraint)
        .setReversed(true)

    // Trajectories
    // Steal Autos
    val stealStartToOpponentTrenchBalls: Trajectory =
        generate(WaypointManager.kStealStart, WaypointManager.kOpponentTrenchBalls, kFwdPickupConfig)

    val opponentTrenchBallsToIntermediate: Trajectory =
        generate(WaypointManager.kOpponentTrenchBalls, WaypointManager.kStealAutoIntermediate, kRevConfig)

    val stealIntermediateToStealScore: Trajectory =
        generate(WaypointManager.kStealAutoIntermediate, WaypointManager.kGoodAfterStealScoringLocation, kFwdConfig)

    val stealScoreToLongTrenchPickup: Trajectory =
        generate(WaypointManager.kGoodAfterStealScoringLocation, WaypointManager.kLongPickupAfterTrench, kFwdPickupConfig)

    // Trench Autos
    val trenchStartToInnerGoalScore: Trajectory =
        generate(WaypointManager.kTrenchStart, WaypointManager.kGoodInnerGoalScoringLocation, kRevConfig)

    val innerGoalScoreToShortTrenchPickup: Trajectory =
        TrajectoryGenerator.generateTrajectory(
            WaypointManager.kGoodInnerGoalScoringLocation, listOf(/*Translation2d(35.41.feet, 3.23.feet)*/),
            WaypointManager.kShortPickupAfterTrench, kFwdPickupConfig
        )

    val innerGoalScoreToLongTrenchPickup: Trajectory =
        TrajectoryGenerator.generateTrajectory(
            WaypointManager.kGoodInnerGoalScoringLocation, listOf(/*Translation2d(35.41.feet, 3.23.feet)*/),
            WaypointManager.kLongPickupAfterTrench, kFwdPickupConfig
        )

    val shortTrenchPickupToTrenchScore: Trajectory =
        generate(WaypointManager.kShortPickupAfterTrench, WaypointManager.kGoodTrenchScoringLocation, kRevConfig)

    val longTrenchPickupToTrenchScore: Trajectory =
        generate(WaypointManager.kLongPickupAfterTrench, WaypointManager.kGoodTrenchScoringLocation, kRevConfig)

    /**
     * Generates a trajectory from a start and end waypoint.
     */
    private fun generate(start: Pose2d, end: Pose2d, config: TrajectoryConfig): Trajectory =
        TrajectoryGenerator.generateTrajectory(
            start, listOf(), end, config
        )
}
