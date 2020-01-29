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
import org.ghrobotics.frc2020.ClimberConstants.kClimberNativeUnitModel
import org.ghrobotics.frc2020.ClimberConstants.kHookId
import org.ghrobotics.frc2020.ClimberConstants.kPistonId
import org.ghrobotics.frc2020.ClimberConstants.kPneumaticModuleId
import org.ghrobotics.frc2020.ClimberConstants.kWinchMasterId
import org.ghrobotics.frc2020.ClimberConstants.kWinchSlaveId
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

    private val frontPiston = Solenoid(kPneumaticModuleId, kPistonId)
    private val backPiston = Solenoid(kPneumaticModuleId, kPistonId)

    private val winchMasterMotor = FalconMAX(
            id = kWinchMasterId,
            type = CANSparkMaxLowLevel.MotorType.kBrushless,
            model = kClimberNativeUnitModel
    )

    private val winchSlaverMotor = FalconMAX(
            id = kWinchSlaveId,
            type = CANSparkMaxLowLevel.MotorType.kBrushless,
            model = kClimberNativeUnitModel
    )

    private val hookMotor = FalconMAX(
            id = kHookId,
            type = CANSparkMaxLowLevel.MotorType.kBrushless,
            model = kClimberNativeUnitModel
    )

    private val periodicIO = PeriodicIO()
    val winchPosition get() = periodicIO.winchPosition
    val hookPosition get() = periodicIO.hookPosition

    private class PeriodicIO() {
        var current: SIUnit<Ampere> = 0.amps
        var voltage: SIUnit<Volt> = 0.volts
        var winchPosition: SIUnit<Meter> = 0.meters
        var hookPosition: SIUnit<Meter> = 0.meters

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
        periodicIO.voltage = winchMasterMotor.voltageOutput
        periodicIO.current = winchMasterMotor.drawnCurrent
        periodicIO.winchPosition = winchMasterMotor.encoder.position
        periodicIO.hookPosition = hookMotor.encoder.position

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> {
                winchMasterMotor.setNeutral()
                hookMotor.setNeutral()
            }
            is Output.Percent ->
                winchMasterMotor.setDutyCycle(desiredOutput.percent, periodicIO.feedforward)

            is Output.Position ->
                winchMasterMotor.setPosition(desiredOutput.position, periodicIO.feedforward)

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
        winchMasterMotor.encoder.resetPosition(position)
    }

    fun hookPercent(percent: Double) {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.HookPercent(percent)
    }

    init {
        winchSlaverMotor.follow(winchMasterMotor)
        defaultCommand = ManualClimberCommand { 0.0 }
        extend(false)
    }

    override fun checkSubsystem(): Command {
        return TestClimberCommand().withTimeout(3.0)
    }
}
