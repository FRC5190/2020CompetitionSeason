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

        println(sensorColor.name)

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
        class Speed(val velocity: SIUnit<Velocity<Meter>>) : Output()
    }

    enum class FortuneColor {
        // Default color value
        BLACK {
            override val rgbValues = rgb(0.0, 0.0, 0.0)
        },

        RED {
            override val rgbValues = rgb(0.562, 0.323, 0.114)
            override fun previous(): FortuneColor{
                return GREEN
            }
        },

        YELLOW {
            override val rgbValues = rgb(0.315, 0.572, 0.112)
        },

        BLUE {
            override val rgbValues = rgb(0.1, 0.422, 0.477)
        },

        GREEN {
            override val rgbValues = rgb(0.143, 0.604, 0.251)
            override fun next(): FortuneColor{
                return RED
            }
        };

        // RGB Values
        protected class rgb(var red: Double, var green: Double, var blue: Double)
        protected abstract val rgbValues: rgb

        val red get() = rgbValues.red
        val green get() = rgbValues.green
        val blue get() = rgbValues.blue

        open fun next(): FortuneColor{
            return values()[ordinal + 1]
        }

        open fun previous(): FortuneColor{
            return values()[ordinal - 1]
        }

        companion object {
            // Returns the fortune wheel color closest to the color given, but only within a designated range.
            fun getFortune(color: Color): FortuneColor {
                var resolution = FortuneWheelConstants.kColorBitDepth

                // Get a lower resolution version of the color to be tested
                var lrColor = Color8Bit((color.red * resolution).roundToInt(), (color.green * resolution).roundToInt(), (color.blue * resolution).roundToInt())

                for (fortune: FortuneColor in values()) {
                    // Get a lower resolution version of the fortune color
                    var lrFortune = Color8Bit((fortune.red * resolution).roundToInt(), (fortune.green * resolution).roundToInt(), (fortune.blue * resolution).roundToInt())

                    if (lrColor.red == lrFortune.red && lrColor.green == lrFortune.green && lrColor.blue == lrFortune.blue) {
                        return fortune
                    }
                }
                return BLACK
            }
        }
    }
}
