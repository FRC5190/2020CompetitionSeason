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
import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.FortuneWheelConstants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.motors.rev.FalconMAX

object FortuneWheel : FalconSubsystem() {
    // Create objects
    private val colorSensor = ColorSensorV3(I2C.Port.kOnboard)
    private val spinnerMotor = FalconMAX(
        id = FortuneWheelConstants.kSpinnerMotorId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = FortuneWheelConstants.kSpinnerUnitModel
    )

    init {
        spinnerMotor.voltageCompSaturation = 12.volts
    }

    // Create PeriodicIO
    private val periodicIO = PeriodicIO()
    val sensorColor get() = periodicIO.sensorColor
    val spinnerPosition get() = periodicIO.spinnerPosition

    override fun setNeutral() {
        periodicIO.desiredOutput = Output.Nothing
    }

    fun setPercent(percent: Double) {
        periodicIO.desiredOutput = Output.Percent(percent)
    }

    // Resets 'position' of encoder
    fun resetPosition() {
        periodicIO.resetPosition = true
    }

    override fun periodic() {
        // Update PeriodicIO variables
        periodicIO.sensorColor = FortuneColor.getFortune(colorSensor.color)
        periodicIO.spinnerPosition = spinnerMotor.encoder.position

        // Do stuff
        if (periodicIO.resetPosition) {
            spinnerMotor.encoder.resetPosition(0.meters)
            periodicIO.resetPosition = false
        }

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> spinnerMotor.setNeutral()
            is Output.Percent -> spinnerMotor.setDutyCycle(desiredOutput.speed)
        }
    }

    override fun checkSubsystem(): Command {
        return TestFortuneWheelCommand().withTimeout(2.0)
    }

    private class PeriodicIO {
        var sensorColor: FortuneColor = FortuneColor.BLACK
        var spinnerPosition: SIUnit<Meter> = 0.meters
        var desiredOutput: Output = Output.Nothing
        var resetPosition: Boolean = false
    }

    private sealed class Output {
        object Nothing : Output()
        class Percent(val speed: Double) : Output()
    }
}
