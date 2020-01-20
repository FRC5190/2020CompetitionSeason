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
import org.ghrobotics.frc2020.ClimberConstants.kClimberMasterId
import org.ghrobotics.frc2020.ClimberConstants.kClimberNativeUnitModel
import org.ghrobotics.frc2020.ClimberConstants.kClimberSlaveId
import org.ghrobotics.frc2020.ClimberConstants.kPistonBrakeId
import org.ghrobotics.frc2020.ClimberConstants.kPistonBrakeModuleId
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

    private val pistonBrake = Solenoid(kPistonBrakeModuleId, kPistonBrakeId)

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
        class ClosedLoop(val Position: SIUnit<Meter>) : Output()
    }

    override fun periodic() {
        periodicIO.voltage = climberMasterMotor.voltageOutput
        periodicIO.current = climberMasterMotor.drawnCurrent
        periodicIO.position = climberMasterMotor.encoder.position

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> {
                climberMasterMotor.setNeutral()
            }
            is Output.Percent -> {
                climberMasterMotor.setDutyCycle(desiredOutput.percent, periodicIO.feedforward)
            }
            is Output.ClosedLoop -> {
                climberMasterMotor.setPosition(desiredOutput.Position, periodicIO.feedforward)
            }
        }
    }

    override fun setNeutral() {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.Nothing
    }

    fun setHeight(desiredHeight: SIUnit<Meter>) {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.ClosedLoop(desiredHeight)
    }

    fun setPercent(percent: Double) {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.Percent(percent)
    }

    fun setBrake(brake: Boolean) {
        pistonBrake.set(brake)
    }

    fun resetPosition(position: SIUnit<Meter>) {
        climberMasterMotor.encoder.resetPosition(position)
    }

    init {
        climberSlaveMotor.follow(climberMasterMotor)
        defaultCommand = ManualClimberCommand { 0.0 }
        pistonBrake.set(true)
    }

    override fun checkSubsystem(): Command {
        return TestClimberCommand().withTimeout(3.0)
    }
}
