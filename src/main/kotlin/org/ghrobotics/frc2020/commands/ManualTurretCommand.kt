package org.ghrobotics.frc2020.commands

import org.ghrobotics.frc2020.subsystems.Turret
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

class ManualTurretCommand(val percent: DoubleSource) : FalconCommand(Turret) {
    override fun execute() {
        Turret.setPercent(percent())
    }

    override fun end(interrupted: Boolean) {
        Turret.setPercent(0.0)
    }
}