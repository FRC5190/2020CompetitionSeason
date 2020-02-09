/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.fortunewheel

import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.util.Color
import org.ghrobotics.frc2020.FortuneWheelConstants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Frac
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.utils.isConnected

object FortuneWheel : FalconSubsystem() {
    // Create objects
    private val colorSensor = ColorSensorV3(I2C.Port.kOnboard)
    private val spinnerMotor = FalconMAX(
        id = FortuneWheelConstants.kSpinnerMotorId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = FortuneWheelConstants.kSpinnerUnitModel
    )

    // Connection Status
    private val isConnected: Boolean

    init {
        isConnected = spinnerMotor.isConnected()

        if (isConnected) {
            spinnerMotor.canSparkMax.restoreFactoryDefaults()
            spinnerMotor.voltageCompSaturation = 12.volts
            spinnerMotor.controller.p = FortuneWheelConstants.kP
            spinnerMotor.controller.ff = FortuneWheelConstants.kF
            spinnerMotor.motionProfileCruiseVelocity = FortuneWheelConstants.kMaxVelocity
            spinnerMotor.motionProfileAcceleration = FortuneWheelConstants.kMaxAcceleration
            spinnerMotor.useMotionProfileForPosition = true
        }
    }

    // Create PeriodicIO
    private val periodicIO = PeriodicIO()
    val sensorColor get() = periodicIO.sensorColor
    val rawColor get() = periodicIO.rawColor
    val spinnerPosition get() = periodicIO.spinnerPosition
    val spinnerVelocity get() = periodicIO.spinnerVelocity

    override fun setNeutral() {
        periodicIO.desiredOutput = Output.Nothing
    }

    fun setPercent(percent: Double) {
        periodicIO.desiredOutput = Output.Percent(percent)
    }

    fun setPosition(position: SIUnit<Meter>) {
        periodicIO.desiredOutput = Output.Position(position)
    }

    // Resets 'position' of encoder
    fun resetPosition() {
        periodicIO.resetPosition = true
    }

    override fun periodic() {
        // Update PeriodicIO variables
        periodicIO.sensorColor = FortuneColor.getFortune(colorSensor.color)
        periodicIO.rawColor = colorSensor.color
        periodicIO.spinnerPosition = spinnerMotor.encoder.position
        periodicIO.spinnerVelocity = spinnerMotor.encoder.velocity

        // Do stuff
        if (periodicIO.resetPosition) {
            spinnerMotor.encoder.resetPosition(0.meters)
            periodicIO.resetPosition = false
        }

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> spinnerMotor.setNeutral()
            is Output.Percent -> spinnerMotor.setDutyCycle(desiredOutput.speed, 0.0.volts)
            is Output.Position -> spinnerMotor.setPosition(desiredOutput.position, 0.0.volts)
        }
    }

    private class PeriodicIO {
        var sensorColor: FortuneColor = FortuneColor.BLACK
        var rawColor: Color = Color(0.0, 0.0, 0.0)
        var spinnerPosition: SIUnit<Meter> = 0.meters
        var spinnerVelocity: SIUnit<Frac<Meter, Second>> = 0.inches / 1.seconds
        var desiredOutput: Output = Output.Nothing
        var resetPosition: Boolean = false
    }

    private sealed class Output {
        object Nothing : Output()
        class Percent(val speed: Double) : Output()
        class Position(val position: SIUnit<Meter>) : Output()
    }
}
