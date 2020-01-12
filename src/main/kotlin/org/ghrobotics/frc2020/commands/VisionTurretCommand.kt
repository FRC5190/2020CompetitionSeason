package org.ghrobotics.frc2020.commands

import org.ghrobotics.frc2020.subsystems.Turret
import org.ghrobotics.frc2020.vision.VisionProcessing
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit

class VisionTurretCommand : FalconCommand(Turret) {
    override fun initialize() {
        VisionProcessing.turnOnLight()
    }
    override fun execute() {
        Turret.setAngle(Turret.angle + SIUnit(VisionProcessing.angle.radians))
    }

    override fun end(interrupted: Boolean) {
        VisionProcessing.turnOffLight()
    }
}