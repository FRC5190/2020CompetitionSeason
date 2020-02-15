package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.auto.TrajectoryManager
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential

class RendezvousTrenchRoutine : AutoRoutine {

    val path1 = TrajectoryManager.trenchStartToNearRendezvous
    val path2 = TrajectoryManager.nearRendezvousToScore

    override fun getRoutine() = sequential {
        +InstantCommand(Runnable { Drivetrain.resetPosition(path1.initialPose) })
        +parallel {
            +Drivetrain.followTrajectory(path1)
            +sequential {
                +WaitCommand(path1.totalTimeSeconds - 1.8)
                +Superstructure.intake()
            }
        }.withTimeout(path1.totalTimeSeconds + 1.0)
        +parallel {
            +Drivetrain.followTrajectory(path2)
           +sequential {
               +WaitCommand(1.5)
               +Superstructure.shoot().withTimeout(path2.totalTimeSeconds - 1.0 + 5.5)
           }
        }
        +parallel {
            +Drivetrain.followTrajectory(TrajectoryManager.pickupAgain)
            +Superstructure.intake()
        }.withTimeout(TrajectoryManager.pickupAgain.totalTimeSeconds + 2.0)
        +Superstructure.shoot()
    }
}