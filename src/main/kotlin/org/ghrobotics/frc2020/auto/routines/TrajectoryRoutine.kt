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
import org.ghrobotics.frc2020.auto.Paths
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential

class TrajectoryRoutine : AutoRoutine {
    override fun getRoutine(): Command {
        return sequential {
            +InstantCommand(Runnable { Drivetrain.resetPosition(Paths.frontOfGoalToTrench.initialPose) })
            +parallelDeadline(Drivetrain.followTrajectory(Paths.frontOfGoalToTrench)) {
                +Superstructure.aimTurret()
            }
            +Superstructure.aimTurret()
        }
    }
}
