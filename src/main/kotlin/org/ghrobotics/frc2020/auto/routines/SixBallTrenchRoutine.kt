/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.auto.TrajectoryManager
import org.ghrobotics.frc2020.auto.WaypointManager
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.turret.AutoTurretCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degrees

class SixBallTrenchRoutine : AutoRoutine {

    // Paths
    private val path1 = TrajectoryManager.trenchStartForwardToScore
    private val path2 = TrajectoryManager.scoreToShortTrenchPickup
    private val path3 = TrajectoryManager.shortTrenchPickupToTrenchScore

    override fun getRoutine() = sequential {

        +InstantCommand(Runnable { Drivetrain.resetPosition(WaypointManager.kTrenchStart) })

        +parallel {
            +Drivetrain.followTrajectory(path1)
            +sequential {
                +AutoTurretCommand { 220.degrees }.withTimeout(1.0)
                +Superstructure.waitUntilStoppedThenShoot()
            }
        }

        // Pickup balls and return to score location whsile aligning
        // to the goal. Then shoot.
        +parallel {
            +sequential {
                +Drivetrain.followTrajectory(path2)
                +Drivetrain.followTrajectory(path3)
            }
            +Superstructure.intakeUntilStoppedThenShoot()
        }
    }
}
