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
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.auto.TrajectoryManager
import org.ghrobotics.frc2020.auto.WaypointManager
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential

class FiveBallStealRoutine : AutoRoutine {

    // Paths
    private val path1 = TrajectoryManager.stealStartToOpponentTrenchBalls
    private val path2 = TrajectoryManager.opponentTrenchBallsToStealScore

    override fun getRoutine(): Command = sequential {
        // Reset odometry.
        +InstantCommand(Runnable { Drivetrain.resetPosition(WaypointManager.kStealStart) })

        // Drive and steal opponent trench balls.
        +parallelDeadline(Drivetrain.followTrajectory(path1)) {
            +Superstructure.intake()
        }

        // Drive to scoring location while aligning to the goal
        // and shoot when path ends.
        +parallel {
            +Drivetrain.followTrajectory(path2)
            +sequential {
                +Superstructure.intake().withTimeout(1.5)
                +Superstructure.waitUntilStoppedThenShoot()
            }
        }
    }
}
