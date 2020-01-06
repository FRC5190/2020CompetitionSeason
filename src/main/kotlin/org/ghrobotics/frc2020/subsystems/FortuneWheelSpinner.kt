/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.roundToInt
import org.ghrobotics.frc2020.FortuneWheelConstants
import org.ghrobotics.frc2020.commands.TestFortuneWheelCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.motors.rev.FalconMAX

object FortuneWheelSpinner : FalconSubsystem() {
    // Create objects
    private val colorSensor = ColorSensorV3(I2C.Port.kOnboard)
    private val spinnerMotor = FalconMAX(
        id = FortuneWheelConstants.kSpinnerMotorId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = FortuneWheelConstants.kSpinnerUnitModel
    )

    // Create PeriodicIO
    private val periodicIO = PeriodicIO()
    val sensorColor get() = periodicIO.sensorColor
    val spinnerPosition get() = periodicIO.spinnerPosition

    // Sets the output speed of the spinner motor
    fun setVelocity(velocity: SIUnit<Velocity<Meter>>) {
        periodicIO.desiredOutput = Output.Speed(velocity)
    }

    // Resets 'position' of encoder
    fun resetPosition() {
        periodicIO.resetPosition = true
    }

    override fun periodic() {
        // Update PeriodicIO variables
        periodicIO.sensorColor = FortuneColor.getFortune(colorSensor.color)
        periodicIO.spinnerPosition = spinnerMotor.encoder.position

        println(colorSensor.color.red)
        println(colorSensor.color.blue)
        println(colorSensor.color.green)

        // Do stuff
        if (periodicIO.resetPosition) {
            spinnerMotor.encoder.resetPosition(0.meters)
            periodicIO.resetPosition = false
        }

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> {
                spinnerMotor.setDutyCycle(0.0)
            }
            is Output.Speed -> {
                spinnerMotor.setVelocity(desiredOutput.velocity)
            }
        }
    }

    override fun checkSubsystem(): Command {
        return TestFortuneWheelCommand()
    }

    private class PeriodicIO {
        var sensorColor: FortuneColor = FortuneColor.Black
        var spinnerPosition: SIUnit<Meter> = 0.meters
        var desiredOutput: Output = Output.Nothing
        var resetPosition: Boolean = false
    }

    private sealed class Output {
        object Nothing : Output()
        class Speed(val velocity: SIUnit<Velocity<Meter>>) : Output()
    }

    // Inefficient, should be replaced soon
    class FortuneColor private constructor(val red: Double, val green: Double, val blue: Double) {
        companion object {
            // Returns this if 'getFortune()' cannot find a match
            val Black get() = FortuneColor(0.0, 0.0, 0.0)

            // Placeholders until I get more accurate numbers
            val Red get() = FortuneColor(1.0, 0.0, 0.0)
            val Yellow get() = FortuneColor(1.0, 1.0, 0.0)
            val Blue get() = FortuneColor(0.0, 0.0, 1.0)
            val Green get() = FortuneColor(0.0, 1.0, 0.0)

            // Order of colors on wheel
            private var colors = arrayOf(Red, Yellow, Blue, Green)

            // Returns the fortune wheel color closest to the color given, but only within a designated range.
            fun getFortune(color: Color): FortuneColor {
                var resolution = FortuneWheelConstants.kColorBitDepth

                // Get a lower resolution version of the color to be tested
                var lrColor = Color8Bit((color.red * resolution).roundToInt(), (color.green * resolution).roundToInt(), (color.blue * resolution).roundToInt())

                for (fortune: FortuneColor in colors) {
                    // Get a lower resolution version of the fortune color
                    var lrFortune = Color8Bit((fortune.red * resolution).roundToInt(), (fortune.green * resolution).roundToInt(), (fortune.blue * resolution).roundToInt())

                    if (lrColor.red == lrFortune.red && lrColor.green == lrFortune.green && lrColor.blue == lrFortune.blue) {
                        return fortune
                    }
                }
                return Black
            }
        }
    }
}
