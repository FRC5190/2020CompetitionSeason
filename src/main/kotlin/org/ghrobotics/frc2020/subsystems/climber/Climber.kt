/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.climber

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.ClimberConstants.kBackPistonId
import org.ghrobotics.frc2020.ClimberConstants.kBackPistonModuleId
import org.ghrobotics.frc2020.ClimberConstants.kClimberMasterId
import org.ghrobotics.frc2020.ClimberConstants.kClimberNativeUnitModel
import org.ghrobotics.frc2020.ClimberConstants.kClimberSlaveId
import org.ghrobotics.frc2020.ClimberConstants.kFrontPistonId
import org.ghrobotics.frc2020.ClimberConstants.kFrontPistonModuleId
import org.ghrobotics.frc2020.ClimberConstants.kHookId
import org.ghrobotics.frc2020.ClimberConstants.kPistonBrakeId
import org.ghrobotics.frc2020.ClimberConstants.kPistonBrakeModuleId
import org.ghrobotics.frc2020.ClimberConstants.kWinchId
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.motors.rev.FalconMAX

object Climber : FalconSubsystem() {

    //private val pistonBrake = Solenoid(kPistonBrakeModuleId, kPistonBrakeId)
    private val frontPiston = Solenoid(kFrontPistonModuleId, kFrontPistonId)
    private val backPiston  = Solenoid(kBackPistonModuleId, kBackPistonId)

    private val winchMotor = FalconMAX(
            id = kWinchId,
            type = CANSparkMaxLowLevel.MotorType.kBrushless,
            model = kClimberNativeUnitModel
    )

    private val hookMotor = FalconMAX(
            id = kHookId,
            type = CANSparkMaxLowLevel.MotorType.kBrushless,
            model = kClimberNativeUnitModel
    )

    /*
    private val climberMasterMotor = FalconMAX(
        id = kClimberMasterId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = kClimberNativeUnitModel
    )

    private val climberSlaveMotor = FalconMAX(
        id = kClimberSlaveId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = kClimberNativeUnitModel
    )
    */


    private val periodicIO = PeriodicIO()
    val position get() = periodicIO.position

    private class PeriodicIO() {
        var current: SIUnit<Ampere> = 0.amps
        var voltage: SIUnit<Volt> = 0.volts
        var position: SIUnit<Meter> = 0.meters

        var feedforward: SIUnit<Volt> = 0.volts
        var desiredOutput: Output = Output.Nothing
    }

    sealed class Output {
        object Nothing : Output()
        class Percent(val percent: Double) : Output()
        class Position(val position: SIUnit<Meter>) : Output()
        class HookPercent(val percent: Double) : Output()
    }

    override fun periodic() {
        periodicIO.voltage = winchMotor.voltageOutput
        periodicIO.current = winchMotor.drawnCurrent
        periodicIO.position = winchMotor.encoder.position

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing ->{
                winchMotor.setNeutral()
                hookMotor.setNeutral()
            }
            is Output.Percent ->
                winchMotor.setDutyCycle(desiredOutput.percent, periodicIO.feedforward)

            is Output.Position ->
                winchMotor.setPosition(desiredOutput.position, periodicIO.feedforward)

            is Output.HookPercent ->
                hookMotor.setDutyCycle(desiredOutput.percent, periodicIO.feedforward)
        }
    }

    override fun setNeutral() {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.Nothing
    }

    fun setHeight(desiredHeight: SIUnit<Meter>) {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.Position(desiredHeight)
    }

    fun setPercent(percent: Double) {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.Percent(percent)
    }

    fun extend(extend: Boolean) {
        frontPiston.set(extend)
        backPiston.set(extend)
    }

    fun resetPosition(position: SIUnit<Meter>) {
        winchMotor.encoder.resetPosition(position)
    }

    fun hookPercent(percent: Double){
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.HookPercent(percent)
    }

    init {
        //climberSlaveMotor.follow(climberMasterMotor)
        defaultCommand = ManualClimberCommand { 0.0 }
        extend(false)
        //pistonBrake.set(true)
    }

    override fun checkSubsystem(): Command {
        return TestClimberCommand().withTimeout(3.0)
    }
}
