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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.auto.TrajectoryManager
import org.ghrobotics.frc2020.auto.WaypointManager
import org.ghrobotics.frc2020.planners.ShooterPlanner
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.hood.HoodConstants
import org.ghrobotics.frc2020.subsystems.hood.HoodPositionCommand
import org.ghrobotics.frc2020.subsystems.shooter.ShooterVelocityCommand
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.frc2020.subsystems.turret.TurretPositionCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degrees

class TrenchRendezvousRoutine : AutoRoutine {
    // First, we will pickup 2 balls in the rendezvous area.
    private val path1 = TrajectoryManager.trenchStartToTrenchRendezvousPickup

    // Then, we will come back to an "intermediate" scoring location to score 5.
    private val path2 = TrajectoryManager.trenchRendezvousPickupToScoringLocation

    // Then, we will pick up the next round of 5 balls.
    private val path3 = TrajectoryManager.intermediateToTrenchPickup

    // Finally, return to the scoring location to score 5.
    private val path4 = TrajectoryManager.trenchPickupToTrenchScoringLocation

    // Now create groups of paths. A group is a collection of paths that basically acts
    // as a single path, with no stops in between.
    private val group1 = path1.concatenate(path2)
    private val group2 = path3.concatenate(path4)

    // The amount of time to feed balls for each score cycle.
    private val kFeedTime = 2.0

    // The shooter and hood parameters for scoring in auto.
    private val kParameters = ShooterPlanner[WaypointManager.kTrenchScoringDistance]

    // At a high level, this "main" sequential command group does 4 things:
    //  * Execute Group 1 of path following while intaking balls.
    //  * Score
    //  * Execute Group 2 of path following while intaking balls.
    //  * Score
    override fun getRoutine(): Command = sequential {
        // Reset odometry to the starting location of the pose.
        +InstantCommand(Runnable { Drivetrain.resetPosition(path1.initialPose) })

        // Execute Group 1 of path following while intaking balls.
        // Note that all commands inside a parallel deadline group are canceled once the
        // deadline command (path following) exits.
        +parallelDeadline(Drivetrain.followTrajectory(group1)) {
            // Always keep track of the inner goal.
            +TurretPositionCommand(Turret.autoBehavior)

            // This sequential command group takes care of the intake behavior.
            +sequential {
                // Wait for a bit and then start the intake.
                val kIntakeWaitTime = 0.5
                +WaitCommand(kIntakeWaitTime)

                // Intake until the first path is complete.
                +Superstructure.intake().withTimeout(path1.totalTimeSeconds - kIntakeWaitTime + 0.5)
            }

            // This sequential command group takes care of the shooter/hood behavior.
            +sequential {
                // Wait for a bit and then start the shooter and hood.
                val kShooterWaitTime = 3.0
                +WaitCommand(kShooterWaitTime)

                // Set the shooter speed and hood angle.
                +parallel {
                    +ShooterVelocityCommand(kParameters.speed)
                    +HoodPositionCommand(kParameters.angle)
                }
            }
        }

        // Score
        +Superstructure.scoreWhenStopped(WaypointManager.kTrenchScoringDistance, feedTime = kFeedTime)

        // Execute Group 2 of path following while intaking balls.
        // Note that all commands inside a parallel deadline group are canceled once the
        // deadline command (path following) exits.
        +parallelDeadline(Drivetrain.followTrajectory(group2)) {
            // Always keep track of the inner goal.
            +TurretPositionCommand(Turret.autoBehavior)

            // Intake until the first path is complete and then some.
            val kFirstPathIntakeOverflowTime = 0.5
            +Superstructure.intake().withTimeout(path3.totalTimeSeconds + kFirstPathIntakeOverflowTime)

            // This sequential command group takes care of the shooter and hood behavior.
            +sequential {
                // Keep the hood down until:
                //  1. The first path has finished (all the balls have been picked up).
                //  2. The robot is out of the control panel region.
                //  3. A certain amount of time has elapsed since (1) and (2) have been met.
                val kSafeTime = 0.2

                // Create the group for the aforementioned conditions.
                val kConditions = sequential {
                    +WaitCommand(path3.totalTimeSeconds)
                    +WaitUntilCommand { !WaypointManager.kControlPanelRegion.contains(Drivetrain.getPose().translation) }
                    +WaitCommand(kSafeTime)
                }

                // Keep the hood down.
                +parallelDeadline(kConditions) {
                    +HoodPositionCommand(HoodConstants.kAcceptableRange.endInclusive - 0.2.degrees)
                }

                // After the hood is safe to go back up again, set the shooter speed and hood angle.
                // Set the shooter speed and hood angle.
                +parallel {
                    +ShooterVelocityCommand(kParameters.speed)
                    +HoodPositionCommand(kParameters.angle)
                }
            }
        }

        // Score
        +Superstructure.scoreWhenStopped(WaypointManager.kTrenchScoringDistance, feedTime = kFeedTime)
    }

    // Returns the duration of this auto routine.
    fun getDuration() = group1.totalTimeSeconds + kFeedTime + group2.totalTimeSeconds + kFeedTime
}
