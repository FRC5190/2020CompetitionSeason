/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.ClimberConstants.kClimberMasterId
import org.ghrobotics.frc2020.ClimberConstants.kClimberNativeUnitModel
import org.ghrobotics.frc2020.ClimberConstants.kClimberSlaveId
import org.ghrobotics.frc2020.ClimberConstants.kPistonBrakeId
import org.ghrobotics.frc2020.ClimberConstants.kPistonBrakeModuleId
import org.ghrobotics.frc2020.commands.ClosedLoopClimberCommand
import org.ghrobotics.frc2020.commands.OpenLoopClimberCommand
import org.ghrobotics.frc2020.commands.TestClimberCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.utils.DoubleSource

object Climber : FalconSubsystem() {

    val pistonBrake = Solenoid(kPistonBrakeModuleId, kPistonBrakeId)

    val ClimberMasterMotor = FalconMAX(
        id = kClimberMasterId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = kClimberNativeUnitModel
        )

    val ClimberSlaveMotor = FalconMAX(
        id = kClimberSlaveId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = kClimberNativeUnitModel
    )

    private val periodicIO = PeriodicIO()
    val position get() = periodicIO.position

    private class PeriodicIO() {
        var current: SIUnit<Ampere> = 0.amps
        var voltage: SIUnit<Volt> = 0.volts
        var position: SIUnit<NativeUnit> = 0.nativeUnits

        var feedforward: SIUnit<Volt> = 0.volts
        var desiredOutput: Output = Output.Nothing
    }

    sealed class Output {
        object Nothing : Output()
        class Percent(val percent: DoubleSource) : Output()
        class ClosedLoop(val height: SIUnit<NativeUnit>, val feedforward: SIUnit<Volt>) : Output()
    }

    override fun periodic() {
        periodicIO.voltage = ClimberMasterMotor.voltageOutput
        periodicIO.current = ClimberMasterMotor.drawnCurrent
        periodicIO.position = ClimberMasterMotor.encoder.position

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> {
                ClimberMasterMotor.setNeutral()
                ClimberSlaveMotor.setNeutral()
            }
            is Output.Percent -> {
                OpenLoopClimberCommand(desiredOutput.percent)
                ClimberMasterMotor.setDutyCycle(desiredOutput.percent(), periodicIO.feedforward)
                ClimberSlaveMotor.setDutyCycle(desiredOutput.percent(), periodicIO.feedforward)
            }
            is Output.ClosedLoop -> {
                ClosedLoopClimberCommand(desiredOutput.height)
                ClimberMasterMotor.setPosition(desiredOutput.height, desiredOutput.feedforward)
                ClimberSlaveMotor.setPosition(desiredOutput.height, desiredOutput.feedforward)
            }
        }
    }

    override fun setNeutral() {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.Nothing
    }

    fun setHeight(desiredHeight: SIUnit<NativeUnit>) {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.ClosedLoop(desiredHeight, periodicIO.feedforward)
    }

    fun setPercent(percent: DoubleSource) {
        periodicIO.feedforward = 0.volts
        periodicIO.desiredOutput = Output.Percent(percent)
    }

    init {
        ClimberSlaveMotor.follow(ClimberMasterMotor)
        defaultCommand = OpenLoopClimberCommand { 0.0 }
        pistonBrake.set(true)
    }

    override fun checkSubsystem(): Command {
        return TestClimberCommand().withTimeout(3.0)
    }
}
