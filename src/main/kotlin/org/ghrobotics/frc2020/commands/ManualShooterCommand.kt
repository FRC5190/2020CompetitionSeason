package org.ghrobotics.frc2020.commands

import org.ghrobotics.frc2020.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

class ManualShooterCommand(val source: DoubleSource) : FalconCommand(Shooter) {
    override fun execute() {
        Shooter.setPercent(source())
    }
}