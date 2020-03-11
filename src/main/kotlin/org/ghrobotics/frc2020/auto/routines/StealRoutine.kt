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
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.auto.TrajectoryManager
import org.ghrobotics.frc2020.auto.WaypointManager
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.turret.TurretPositionCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degrees

class StealRoutine(private val shootFromProtected: Boolean = false) : AutoRoutine {

    // Paths
    private val path1 = TrajectoryManager.stealStartToOpponentTrenchBalls

    private val path2 = if (shootFromProtected) {
        TrajectoryManager.opponentTrenchBallsToProtectedScoringLocation
    } else {
        TrajectoryManager.opponentTrenchBallsToInitLineScoringLocation
    }

    private val path3 = if (shootFromProtected) {
        TrajectoryManager.protectedScoringLocationToDoubleRendezvousPickup
    } else {
        TrajectoryManager.initLineScoringLocationToDoubleRendezvousPickup
    }

    private val path4 = TrajectoryManager.doubleRendezvousPickupToRendezvousIntermediate
    private val path5 = TrajectoryManager.rendezvousIntermediateToSingleRendezvousPickup

    private val path6 = if (shootFromProtected) {
        TrajectoryManager.singleRendezvousPickupToProtectedScoringLocation
    } else {
        TrajectoryManager.singleRendezvousPickupToInitLineScoringLocation
    }

    override fun getRoutine(): Command = sequential {
        // Reset odometry.
        +InstantCommand(Runnable { Drivetrain.resetPosition(WaypointManager.kStealStart) })

        // Pickup balls.
        +parallelDeadline(Drivetrain.followTrajectory(path1)) {
            +Superstructure.intake()
        }

        // Score from specified location.
        +parallel {
            +Drivetrain.followTrajectory(path2)
            +sequential {
                +parallel {
                    +TurretPositionCommand { 150.degrees }
                    +Superstructure.intake()
                }.withTimeout(path2.totalTimeSeconds - 1.5)
                +Superstructure.scoreWhenStopped(
                    distance = if (shootFromProtected) WaypointManager.kInitLineScoringDistance else
                        WaypointManager.kProtectedScoringDistance, feedTime = 1.8
                )
            }
        }

        // Pickup three other balls.
        +parallelDeadline(Drivetrain.followTrajectory(path3)) {
            +sequential {
                +WaitCommand(path3.totalTimeSeconds - 0.3)
                +Superstructure.intake()
            }
        }

        +Drivetrain.followTrajectory(path4).deadlineWith(Superstructure.intake())
        // Pickup three other balls.
        +parallelDeadline(Drivetrain.followTrajectory(path5)) {
            +sequential {
                +WaitCommand(path5.totalTimeSeconds - 0.3)
                +Superstructure.intake()
            }
        }

        // Score from specified location.
        +parallel {
            +Drivetrain.followTrajectory(path6)
            +sequential {
                +parallel {
                    +TurretPositionCommand { 150.degrees }
                    +Superstructure.intake()
                }.withTimeout(1.0)
                +Superstructure.scoreWhenStopped(
                    distance = if (shootFromProtected) WaypointManager.kInitLineScoringDistance else
                        WaypointManager.kProtectedScoringDistance, feedTime = 1.8
                )
            }
        }
    }

    fun getDuration(): Double =
        path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds +
            path4.totalTimeSeconds + path5.totalTimeSeconds + path6.totalTimeSeconds
}
