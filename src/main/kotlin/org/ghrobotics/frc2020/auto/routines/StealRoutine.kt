package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.auto.TrajectoryManager
import org.ghrobotics.frc2020.auto.WaypointManager
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.sequential

class StealRoutine(private val long: Boolean) : AutoRoutine {

    override fun getRoutine(): Command = sequential {
        +InstantCommand(Runnable { Drivetrain.resetPosition(WaypointManager.kStealStart) })
        +Drivetrain.followTrajectory(TrajectoryManager.stealStartToOpponentTrenchBalls)
        +Drivetrain.followTrajectory(TrajectoryManager.opponentTrenchBallsToScore)

        if (long) {
            +Drivetrain.followTrajectory(TrajectoryManager.scoreToLongTrench)
            +Drivetrain.followTrajectory(TrajectoryManager.longTrenchToShortTrench)
        } else {
            +Drivetrain.followTrajectory(TrajectoryManager.scoreToShortTrench)
        }
    }

    fun getTime() = TrajectoryManager.stealStartToOpponentTrenchBalls.totalTimeSeconds +
        TrajectoryManager.opponentTrenchBallsToScore.totalTimeSeconds +
        if (long) {
            TrajectoryManager.scoreToLongTrench.totalTimeSeconds +
                TrajectoryManager.longTrenchToShortTrench.totalTimeSeconds
        } else {
            TrajectoryManager.scoreToShortTrench.totalTimeSeconds
        }


}