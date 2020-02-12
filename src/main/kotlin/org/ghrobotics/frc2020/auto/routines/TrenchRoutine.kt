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
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.inSeconds
import org.ghrobotics.lib.mathematics.units.seconds

class TrenchRoutine(private val type: Type) : AutoRoutine {

    // Constants
    private val kIntakeDelayTolerance = 0.3.seconds

    // Paths
    private val path1 =
        if (type == Type.SIX_BALL) TrajectoryManager.trenchStartToShortPickup else TrajectoryManager.trenchStartToLongPickup
    private val path2 = TrajectoryManager.longPickupToShortPickup

    /**
     * Returns the command that runs the auto routine.
     * @return The command that runs the auto routine.
     */
    override fun getRoutine(): Command = sequential {
        // Reset odometry
        +InstantCommand(Runnable { Drivetrain.resetPosition(WaypointManager.kTrenchStart) })

        // Shoot existing power cells from current location.
        +Superstructure.shoot()

        // Pickup more power cells.
        +parallel {
            +Drivetrain.followTrajectory(path1)
            +Superstructure.intake()
        }.withTimeout(path1.totalTimeSeconds + kIntakeDelayTolerance.inSeconds())

        // Shot the power cells if it's a 6 ball auto. Come back and
        // shoot and if 8 ball.
        if (type == Type.SIX_BALL) {
            +Superstructure.shoot()
        } else {
            +parallel {
                +Drivetrain.followTrajectory(path2)
                +sequential {
                    +WaitCommand(0.2)
                    +Superstructure.shoot()
                }
            }
        }
    }

    enum class Type {
        SIX_BALL, EIGHT_BALL
    }
}
