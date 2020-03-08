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
import org.ghrobotics.lib.mathematics.units.inches

class TrenchRoutine : AutoRoutine {

    // Paths
    private val path1 = TrajectoryManager.trenchStartToTrenchRendezvousPickup
    private val path2 = TrajectoryManager.trenchRendezvousPickupToIntermediate
    private val path3 = TrajectoryManager.intermediateToTrenchPickup
    private val path4 = TrajectoryManager.trenchPickupToTrenchScoringLocation

    private val firstVolleyParameters = ShooterPlanner[WaypointManager.kTrenchRedezvousScoringDistance]

    override fun getRoutine(): Command = sequential {
        // Reset odometry.
        +InstantCommand(Runnable { Drivetrain.resetPosition(WaypointManager.kTrenchStart) })

        // Follow path and intake.
        +parallel {
            +Drivetrain.followTrajectory(path1)
            +sequential {
                +TurretPositionCommand { -115.degrees }.withTimeout(0.5)
                +TurretPositionCommand(Turret.defaultBehavior)
            }
            +sequential {
                +WaitCommand(0.2)
                +parallel {
                    +ShooterVelocityCommand(firstVolleyParameters.speed)
                    +HoodPositionCommand(firstVolleyParameters.angle)
                }
            }
            +sequential {
                +WaitCommand(path1.totalTimeSeconds - 0.3)
                +Superstructure.intake()
            }
        }.withTimeout(path1.totalTimeSeconds + 0.85)

        // Shoot
        +Superstructure.scoreWhenStopped(distance = WaypointManager.kTrenchRedezvousScoringDistance + 5.0.inches, feedTime = 2.3)

        // Go to intermediate.
        +Drivetrain.followTrajectory(path2)

        // Pickup balls.
        +parallelDeadline(Drivetrain.followTrajectory(path3)) {
            +Superstructure.intake()
            +HoodPositionCommand { HoodConstants.kAcceptableRange.endInclusive - 0.2.degrees }
            +TurretPositionCommand { 210.degrees }
        }

        // Score balls.
        +parallel {
            +Drivetrain.followTrajectory(path4)
            +sequential {
                +WaitUntilCommand { !WaypointManager.kControlPanelRegion.contains(Drivetrain.getPose().translation) }
                +Superstructure.scoreWhenStopped(distance = WaypointManager.kTrenchScoringDistance, feedTime = 2.5)
            }
        }
    }

    fun getPathDuration(): Double =
        path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds + path4.totalTimeSeconds
}
