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
import org.ghrobotics.frc2020.subsystems.hood.AutoHoodCommand
import org.ghrobotics.frc2020.subsystems.turret.AutoTurretCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential

class EightBallTrenchRoutine : AutoRoutine {

    // Paths
    private val path1 = TrajectoryManager.trenchStartToInnerGoalScore
    private val path2 = TrajectoryManager.innerGoalScoreToLongTrenchPickup
    private val path3 = TrajectoryManager.longTrenchPickupToTrenchScore

    /**
     * Returns the command that runs the auto routine.
     * @return The command that runs the auto routine.
     */
    override fun getRoutine(): Command = sequential {
        // Reset odometry
        +InstantCommand(Runnable { Drivetrain.resetPosition(WaypointManager.kTrenchStart) })

        // Follow trajectory while aligning, and shot balls at the end.
        +parallel {
            +Drivetrain.followTrajectory(path1)
            +Superstructure.waitUntilStoppedThenShoot(1.5)
        }

        // Pickup balls and return to score location while aligning
        // to the goal. Then shoot.
        +parallelDeadline(Drivetrain.followTrajectory(path2)) {
            +Superstructure.intake()
            +AutoTurretCommand.createFromFieldOrientedAngle(Rotation2d())
            +AutoHoodCommand { HoodConstants.kAcceptableRange.endInclusive }
        }

        +parallel {
            +Drivetrain.followTrajectory(path3)
            +sequential {
                +parallel {
                    +Superstructure.intake(1.0)
                    +AutoTurretCommand.createFromFieldOrientedAngle(Rotation2d())
                }.withInterrupt { !WaypointManager.kControlPanelRegion.contains(Drivetrain.getPose().translation) }
                +Superstructure.waitUntilStoppedThenShoot()
            }
        }
    }

    fun getPathDuration(): Double =
        path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds
}
