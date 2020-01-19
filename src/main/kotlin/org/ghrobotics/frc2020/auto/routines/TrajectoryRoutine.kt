package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.auto.Paths
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential

class TrajectoryRoutine : AutoRoutine {
    override fun getRoutine(): Command {
        return sequential {
            +parallel {
                +InstantCommand(Runnable { Drivetrain.resetPosition(Paths.frontOfGoalToTrench.initialPose) })
                +Drivetrain.followTrajectory(Paths.frontOfGoalToTrench)
                +Superstructure.aimTurret()
            }
            +Superstructure.aimTurret()
        }
    }
}