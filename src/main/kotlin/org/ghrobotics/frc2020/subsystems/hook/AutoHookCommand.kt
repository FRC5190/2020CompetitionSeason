package org.ghrobotics.frc2020.subsystems.hook

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit

class AutoHookCommand(private val desiredPosition: SIUnit<Meter>): FalconCommand(Hook){

    override fun execute() {
        Hook.setPosition(desiredPosition)
    }

    override fun end(interrupted: Boolean) {
        Hook.setNeutral()
    }
}