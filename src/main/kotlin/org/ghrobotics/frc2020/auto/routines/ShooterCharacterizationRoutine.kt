package org.ghrobotics.frc2020.auto.routines

import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.subsystems.shooter.CharacterizeShooterCommand

class ShooterCharacterizationRoutine : AutoRoutine {
    override fun getRoutine() = CharacterizeShooterCommand()
}