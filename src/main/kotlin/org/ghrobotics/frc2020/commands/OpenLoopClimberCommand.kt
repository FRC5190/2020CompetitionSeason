package org.ghrobotics.frc2020.commands

import org.ghrobotics.frc2020.subsystems.Climber
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

class OpenLoopClimberCommand(private val percentSource : DoubleSource) : FalconCommand(Climber){
    //do brake stuff here
    override fun execute() {
        Climber.setPercent(percentSource())
    }

    override fun end(interrupted: Boolean) {
        Climber.setNeutral()
    }
}