package org.ghrobotics.frc2020.subsystems.hook

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.HookConstants.kHookId
import org.ghrobotics.frc2020.HookConstants.kHookNativeUnitModel
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.motors.rev.FalconMAX

object Hook: FalconSubsystem() {

    private val hookMotor = FalconMAX(
            id = kHookId,
            type = CANSparkMaxLowLevel.MotorType.kBrushless,
            model = kHookNativeUnitModel
    )

    private val periodicIO = PeriodicIO()
    val hookPosition get() = periodicIO.position

    private class PeriodicIO(){
        var current: SIUnit<Ampere> = 0.amps
        var voltage: SIUnit<Volt> = 0.volts
        var position: SIUnit<Meter> = 0.meters

        var feedforward: SIUnit<Volt> = 0.volts
        var desiredOutput: Output = Output.Nothing

    }

    override fun periodic(){
        periodicIO.voltage = hookMotor.voltageOutput
        periodicIO.current = hookMotor.drawnCurrent
        periodicIO.position = hookMotor.encoder.position

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing ->
                hookMotor.setNeutral()
            is Output.Percent ->
                hookMotor.setDutyCycle(desiredOutput.percent, periodicIO.feedforward)
            is Output.Position ->
                hookMotor.setPosition(desiredOutput.position, periodicIO.feedforward)
        }
    }

    override fun setNeutral() {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.Nothing
    }
    
    fun setPercent(percent: Double){
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.Percent(percent)
    }

    fun resetPosition(position: SIUnit<Meter>) {
        hookMotor.encoder.resetPosition(position)
    }

    fun setPosition(position: SIUnit<Meter>) {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.Position(position)
    }

    init{
        defaultCommand = ManualHookCommand { 0.0 }
    }

    sealed class Output{
        object Nothing: Output()
        class Percent(val percent: Double) : Output()
        class Position(val position: SIUnit<Meter>): Output()
    }

    override fun checkSubsystem(): Command {
        return TestHookCommand().withTimeout(3.0)
    }
}