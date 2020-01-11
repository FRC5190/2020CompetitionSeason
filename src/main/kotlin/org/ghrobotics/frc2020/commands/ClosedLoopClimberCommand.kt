package org.ghrobotics.frc2020.commands

import org.ghrobotics.frc2020.subsystems.Climber
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit

class ClosedLoopClimberCommand(private val desiredHeight : SIUnit<NativeUnit>) : FalconCommand(Climber){

    //do break stuff
    override fun initialize() {
        Climber.setHeight(desiredHeight)
    }

    override fun end(interrupted: Boolean) {
        Climber.ClimberMasterMotor.setNeutral()
        Climber.ClimberSlaveMotor.setNeutral()
    }
}