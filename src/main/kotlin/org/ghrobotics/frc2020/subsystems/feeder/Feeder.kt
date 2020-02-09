/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.feeder

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.AnalogInput
import org.ghrobotics.frc2020.FeederConstants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.utils.isConnected

object Feeder : FalconSubsystem() {
    // Create objects
    private val intakeSensor = AnalogInput(FeederConstants.kIntakeSensorId)
    private val turretSensor = AnalogInput(FeederConstants.kTurretSensorId)
    private val feederMotor = FalconMAX(
        id = FeederConstants.kFeederMasterId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = FeederConstants.kFeederUnitModel
    )

    // Create PeriodicIO
    private val periodicIO = PeriodicIO()
    val intakeValue get() = periodicIO.intakeValue
    val turretValue get() = periodicIO.turretValue

    // Feeder Values
    var ballCount = 0
    var status = Status.INTAKE

    // Connection Status
    private val isConnected: Boolean

    init {
        isConnected = feederMotor.isConnected()
        if (isConnected) {
            feederMotor.outputInverted = true
            feederMotor.smartCurrentLimit = FeederConstants.kCurrentLimit
        }
    }

    fun setPercent(percent: Double) {
        periodicIO.output = Output.Percent(percent)
    }

    fun setPosition(position: SIUnit<Meter>) {
        periodicIO.output = Output.Position(position)
    }

    override fun periodic() {
        if (isConnected) {
            periodicIO.intakeValue = intakeSensor.value != FeederConstants.kNormalIntake
            periodicIO.turretValue = turretSensor.value != FeederConstants.kNormalTurret

            when (val output = periodicIO.output) {
                is Output.Nothing -> feederMotor.setNeutral()
                is Output.Percent -> feederMotor.setDutyCycle(output.percent)
                is Output.Position -> feederMotor.setPosition(output.position)
            }
        }
    }

    private class PeriodicIO {
        var output: Output = Output.Nothing
        var intakeValue: Boolean = false
        var turretValue: Boolean = false
    }

    private sealed class Output {
        object Nothing : Output()
        class Percent(val percent: Double) : Output()
        class Position(val position: SIUnit<Meter>) : Output()
    }

    enum class Status {
        INTAKE,
        TURRET,
        TOINTAKE
    }
}
