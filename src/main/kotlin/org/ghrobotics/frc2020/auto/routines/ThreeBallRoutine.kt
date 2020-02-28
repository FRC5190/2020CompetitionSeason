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
import org.ghrobotics.lib.commands.sequential

class ThreeBallRoutine : AutoRoutine {

    private val path1 = TrajectoryManager.trenchStartReversedToInnerGoalScore

    override fun getRoutine(): Command = sequential {
        // Reset odometry
        +InstantCommand(Runnable { Drivetrain.resetPosition(WaypointManager.kTrenchReverseStart) })

        // Follow trajectory while aligning, and shot balls at the end.
        +parallel {
            +Drivetrain.followTrajectory(path1)
            +Superstructure.waitUntilStoppedThenShoot()
        }
    }

    fun getPathDuration(): Double =
        path1.totalTimeSeconds
}
