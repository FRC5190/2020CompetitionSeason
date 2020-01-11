package org.ghrobotics.frc2020.commands

import org.ghrobotics.frc2020.subsystems.Climber
import org.ghrobotics.frc2020.subsystems.Intake
import org.ghrobotics.frc2020.subsystems.SubsystemTestManager
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

class TestClimberCommand : FalconCommand(Climber){
    private var success = true
    //need to add break stuff
    override fun initialize(){
        Climber.ClimberMasterMotor.encoder.resetPosition(0.nativeUnits)
        Climber.ClimberSlaveMotor.encoder.resetPosition(0.nativeUnits)
    }
    override fun execute() {
        OpenLoopClimberCommand{.75}
    }

    override fun end(interrupted: Boolean) {
        success = Climber.position > 3.nativeUnits
        Climber.setNeutral()
        SubsystemTestManager.climberCheck = success
    }
}