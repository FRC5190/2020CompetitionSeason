package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential

class ThreeBallRoutine : AutoRoutine {
    override fun getRoutine(): Command = sequential {
        +parallel {
//            +Drivetrain.followTrajectory(TrajectoryManager.startToOpenScoringLocation)
            +Superstructure.aimTurret()
        }
    }
}