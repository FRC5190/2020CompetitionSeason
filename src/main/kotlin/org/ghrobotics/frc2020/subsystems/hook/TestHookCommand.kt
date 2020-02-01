package org.ghrobotics.frc2020.subsystems.hook

import org.ghrobotics.frc2020.subsystems.SubsystemTestManager
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.meters

class TestHookCommand : FalconCommand(Hook){
    private var success = false

    override fun initialize() {
        Hook.resetPosition(0.meters)
    }

    override fun execute() {
        Hook.setPercent(.75)
    }

    override fun end(interrupted: Boolean) {
        success = Hook.hookPosition > 1.meters
        Hook.setNeutral()
        SubsystemTestManager.hookCheck = success
    }


}