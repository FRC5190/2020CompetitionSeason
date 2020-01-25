package org.ghrobotics.frc2020.subsystems.climber

import org.ghrobotics.lib.commands.FalconCommand

class HookCommand : FalconCommand(Climber){

    override fun execute() {
        Climber.hookPercent(.75)
    }

    override fun end(interrupted: Boolean) {
        Climber.setNeutral()
    }
}