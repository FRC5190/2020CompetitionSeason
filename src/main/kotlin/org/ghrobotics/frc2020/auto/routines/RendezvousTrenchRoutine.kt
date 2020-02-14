package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.auto.TrajectoryManager
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential

class RendezvousTrenchRoutine : AutoRoutine {

    val path1 = TrajectoryManager.trenchStartToNearRendezvous

    override fun getRoutine() = sequential {
        +parallel {
            +Drivetrain.followTrajectory(path1)
            +Superstructure.intake()
        }.withTimeout(path1.totalTimeSeconds + 0.2)
        +Superstructure.shoot()
    }
}