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
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2020.FeederConstants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.utils.isConnected

object Feeder : FalconSubsystem() {
    // Create objects
    private val feederMotor = FalconMAX(
        id = FeederConstants.kFeederId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = FeederConstants.kFeederUnitModel
    )
    private val bridgeMotor = FalconMAX(
        id = FeederConstants.kBridgeId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = DefaultNativeUnitModel
    )

    private val exitPiston = Solenoid(FeederConstants.kPCMId, FeederConstants.kExitPistonId)

    private val intakeSensor = AnalogInput(FeederConstants.kIntakeSensorId)
    private val exitSensor = AnalogInput(FeederConstants.kExitSensorId)

    // Create PeriodicIO
    private val periodicIO = PeriodicIO()

    // Connection Status
    private var isConnected = false

    var exitPistons = false

    // Getters
    val intakeSensorTriggered get() = periodicIO.intakeSensor
    val exitSensorTriggered get() = periodicIO.exitSensor

    override fun lateInit() {
        isConnected = feederMotor.isConnected() && bridgeMotor.isConnected()
        if (isConnected) {
            feederMotor.canSparkMax.restoreFactoryDefaults()
            bridgeMotor.canSparkMax.restoreFactoryDefaults()

            feederMotor.outputInverted = true
            bridgeMotor.outputInverted = true

            feederMotor.smartCurrentLimit = FeederConstants.kCurrentLimit
            bridgeMotor.smartCurrentLimit = FeederConstants.kCurrentLimit

            setExitPiston(false)
        } else {
            println("Did not initialize Feeder")
        }
    }

    fun setPercent(feederPercent: Double, bridgePercent: Double) {
        periodicIO.output = Output.Percent(feederPercent, bridgePercent)
    }

    fun setExitPiston(extend: Boolean) {
        exitPiston.set(extend)
        exitPistons = extend
    }

    override fun periodic() {
        if (isConnected) {
            periodicIO.intakeSensor = intakeSensor.averageVoltage > 1.25
            periodicIO.exitSensor = exitSensor.averageVoltage > 1.0

            when (val output = periodicIO.output) {
                is Output.Nothing -> {
                    feederMotor.setNeutral()
                    bridgeMotor.setNeutral()
                }
                is Output.Percent -> {
                    feederMotor.setDutyCycle(output.feederPercent)
                    bridgeMotor.setDutyCycle(output.bridgePercent)
                }
            }
        }
    }

    override fun setNeutral() {
        periodicIO.output = Output.Nothing
    }

    private class PeriodicIO {
        var output: Output = Output.Nothing

        var intakeSensor: Boolean = false
        var exitSensor: Boolean = false
    }

    private sealed class Output {
        object Nothing : Output()
        class Percent(val feederPercent: Double, val bridgePercent: Double) : Output()
    }
}
