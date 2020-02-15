package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.auto.TrajectoryManager
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential

/**
 * 10 ball auto routine that picks up balls at the rendezvous point
 * and the trench.
 */
class RendezvousTrenchRoutine : AutoRoutine {

    private val path1 = TrajectoryManager.trenchStartToNearRendezvous
    private val path2 = TrajectoryManager.nearRendezvousToScore
    private val path3 = TrajectoryManager.pickupAgain

    private val kIntakeDropTime = 1.0 // seconds
    private val kStartAimingDelay = 1.0 // seconds
    private val kShootingTime = 2.0 // seconds

    override fun getRoutine() = sequential {
        // Reset odometry
        +InstantCommand(Runnable { Drivetrain.resetPosition(path1.initialPose) })

        // Follow first trajectory and intake 2 balls while aligning to the goal.
        +parallelDeadline(sequential { +WaitCommand(kStartAimingDelay); +Superstructure.aimToGoal() }) {
            // Drive trajectory.
            +Drivetrain.followTrajectory(path1)
            // Wait until we get to the drop time and start intaking.
            +sequential {
                +WaitCommand(path1.totalTimeSeconds - kIntakeDropTime)
                +Superstructure.intake()
            }
        }

        // Shoot into goal.
        +Superstructure.shootIntoGoal().withTimeout(kShootingTime)

        // Pickup trench balls while aiming to goal.
        +Drivetrain.followTrajectory(path2)
        +parallelDeadline(sequential { +WaitCommand(kStartAimingDelay); +Superstructure.aimToGoal() }) {
            +Drivetrain.followTrajectory(path3)
            +Superstructure.intake()
        }

        // Shoot into goal.
        +Superstructure.shootIntoGoal().withTimeout(kShootingTime)
    }

    fun getTime(): Double {
        return path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds +
            2 * kShootingTime + 2 * 0.7
    }
}