package org.ghrobotics.frc2020.subsystems.climber

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

class HookCommand(private val percentSource: DoubleSource) : FalconCommand(Climber){

    override fun execute() {
        Climber.hookPercent(percentSource())
    }

    override fun end(interrupted: Boolean) {
        Climber.setNeutral()
    }
}