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

class StealRoutine(private val type: Type) : AutoRoutine {

    // Constants
    private val kIntakeDelayTolerance = 0.3.seconds

    // Paths
    private val path1 = TrajectoryManager.stealStartToOpponentTrenchBalls
    private val path2 = TrajectoryManager.opponentTrenchBallsToStealScore
    private val path3 =
        if (type == Type.EIGHT_BALL) TrajectoryManager.stealScoreToShortPickup else TrajectoryManager.stealScoreToLongPickup
    private val path4 = TrajectoryManager.longPickupToShortPickup

    /**
     * Returns the command that runs the auto routine.
     * @return The command that runs the auto routine.
     */
    override fun getRoutine(): Command = sequential {
        // Reset odometry to current location.
        +InstantCommand(Runnable { Drivetrain.resetPosition(WaypointManager.kStealStart) })

        // Drive and pickup balls from opponent trench.
        +parallel {
            +Drivetrain.followTrajectory(path1)
            +Superstructure.intake()
        }.withTimeout(path1.totalTimeSeconds + kIntakeDelayTolerance.inSeconds())

        // Drive back to the scoring position and score.
        +parallel {
            +Drivetrain.followTrajectory(path2)
            +sequential {
                +WaitCommand(0.2)
                +Superstructure.shoot()
            }
        }

        // Pickup more balls.
        +parallel {
            +Drivetrain.followTrajectory(path3)
            +Superstructure.intake()
        }.withTimeout(path3.totalTimeSeconds + kIntakeDelayTolerance.inSeconds())

        // Score immediately if 8 ball auto, or come back into range
        // if 10 ball auto.
        if (type == Type.EIGHT_BALL) {
            +Superstructure.shoot()
        } else {
            +parallel {
                +Drivetrain.followTrajectory(path4)
                +sequential {
                    +WaitCommand(0.2)
                    +Superstructure.shoot()
                }
            }
        }
    }

    enum class Type {
        EIGHT_BALL, TEN_BALL
    }
}
