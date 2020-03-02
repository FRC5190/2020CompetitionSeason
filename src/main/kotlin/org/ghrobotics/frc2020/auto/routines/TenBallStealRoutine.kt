/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.ghrobotics.frc2020.HoodConstants
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.auto.TrajectoryManager
import org.ghrobotics.frc2020.auto.WaypointManager
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.hood.HoodPositionCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential

class TenBallStealRoutine : AutoRoutine {

    // Paths
    private val path1 = TrajectoryManager.stealStartToOpponentTrenchBalls
    private val path2 = TrajectoryManager.opponentTrenchBallsToIntermediate
    private val path3 = TrajectoryManager.stealIntermediateToStealScore
    private val path4 = TrajectoryManager.stealScoreToLongTrenchPickup
    private val path5 = TrajectoryManager.longTrenchPickupToTrenchScore

    override fun getRoutine(): Command = sequential {
        // Reset odometry
        +InstantCommand(Runnable { Drivetrain.resetPosition(WaypointManager.kStealStart) })

        // Intake opponent trench balls and come forward a little bit.
        +parallelDeadline(sequential {
            +Drivetrain.followTrajectory(path1)
            +Drivetrain.followTrajectory(path2)
        }) {
            +Superstructure.intake()
        }

        // Go to scoring position and shoot.
        +parallel {
            +Drivetrain.followTrajectory(path3)
            +sequential {
                +AutoTurretCommand.createFromFieldOrientedAngle(Rotation2d()).withTimeout(1.5)
                +Superstructure.waitUntilStoppedThenShoot()
            }
        }

        // Pickup trench balls.
        +parallelDeadline(Drivetrain.followTrajectory(path4)) {
            +Superstructure.intake()
            +AutoTurretCommand.createFromFieldOrientedAngle(Rotation2d())
            +HoodPositionCommand { HoodConstants.kAcceptableRange.endInclusive }
        }

        // Return to scoring location, then shoot.
        +parallel {
            +Drivetrain.followTrajectory(path5)
            +sequential {
                +parallel {
                    +Superstructure.intake()
                    +AutoTurretCommand.createFromFieldOrientedAngle(Rotation2d())
                }.withInterrupt { !WaypointManager.kControlPanelRegion.contains(Drivetrain.getPose().translation) }
                +Superstructure.waitUntilStoppedThenShoot()
            }
        }
    }

    fun getPathDuration(): Double =
        path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds +
            path4.totalTimeSeconds + path5.totalTimeSeconds
}
