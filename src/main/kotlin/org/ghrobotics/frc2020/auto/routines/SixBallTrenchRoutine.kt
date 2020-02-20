/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.auto.TrajectoryManager
import org.ghrobotics.frc2020.auto.WaypointManager
import org.ghrobotics.frc2020.planners.ShooterPlanner
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SIUnit

class SixBallTrenchRoutine(private val pushOff: Boolean = false) : AutoRoutine {

    // Paths
    private val path1 = TrajectoryManager.trenchStartToInnerGoalScore
    private val path2 = TrajectoryManager.innerGoalScoreToShortTrenchPickup
    private val path3 = TrajectoryManager.shortTrenchPickupToTrenchScore

    // Constants
    private val kPushTime = 0.3
    private val kPushSpeed = -0.2
    private val kStartInitialVisionAlign = 1.6

    // Shooting parameters for initial shot.
    private val initialShootingParams =
        ShooterPlanner[SIUnit(
            WaypointManager.kGoodInnerGoalScoringLocation.translation.getDistance(
                VisionConstants.kGoalLocation.translation
            )
        )]

    /**
     * Returns the command that runs the auto routine.
     * @return The command that runs the auto routine.
     */
    override fun getRoutine(): Command = sequential {
        // Reset odometry
        +InstantCommand(Runnable { Drivetrain.resetPosition(WaypointManager.kTrenchStart) })

        // Push alliance partner (if needed).
        if (pushOff) {
            +object : FalconCommand(Drivetrain) {
                override fun initialize() = Drivetrain.setPercent(kPushSpeed, kPushSpeed)
            }.withTimeout(kPushTime)
        }

        // Follow trajectory while aligning, and shot balls at the end.
        +parallel {
            +Drivetrain.followTrajectory(path1)
            +Superstructure.waitUntilStoppedThenShoot(2.0)
        }

        // Pickup balls and return to score location while aligning
        // to the goal. Then shoot.
        +parallel {
            +sequential {
                +Drivetrain.followTrajectory(path2)
                +Drivetrain.followTrajectory(path3)
            }
            +Superstructure.intakeUntilStoppedThenShoot()
        }
    }

    fun getPathDuration(): Double =
        path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds
}
