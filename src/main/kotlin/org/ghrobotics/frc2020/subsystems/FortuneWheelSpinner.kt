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
import org.ghrobotics.frc2020.commands.FortuneWheelPositionCommand
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
    val spinnerMotor = FalconMAX(
        id = FortuneWheelConstants.kSpinnerMotorId,
        type = CANSparkMaxLowLevel.MotorType.kBrushless,
        model = FortuneWheelConstants.kSpinnerUnitModel
    )

    // Create PeriodicIO
    private val periodicIO = PeriodicIO()
    val sensorColor get() = periodicIO.sensorColor
    val spinnerPosition get() = periodicIO.spinnerPosition

    fun setPercent(percent: Double) {
        periodicIO.desiredOutput = Output.Percent(percent)
    }

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

        println(periodicIO.sensorColor)
        // println("R: " + colorSensor.color.red + " G: " + colorSensor.color.green + " B: " + colorSensor.color.blue)

        // Do stuff
        if (periodicIO.resetPosition) {
            spinnerMotor.encoder.resetPosition(0.meters)
            periodicIO.resetPosition = false
        }

        when (val desiredOutput = periodicIO.desiredOutput) {
            is Output.Nothing -> spinnerMotor.setDutyCycle(0.0)
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
        class Speed(val velocity: SIUnit<Velocity<Meter>>) : Output()
        class Percent(val speed: Double) : Output()
    }

    init {

    }

    enum class FortuneColor {
        // Default color value
        BLACK {
            override val rgb = RGB(0.0, 0.0, 0.0)
            override fun next(): FortuneColor {
                return BLACK
            }
            override fun previous(): FortuneColor {
                return BLACK
            }
        },

        // Fortune Wheel colors
        RED {
            // old values r: 0.562 g: 0.323 b: 0.114
            // correct values r: 0.3842 g: 0.4152 b: 0.2004
            override val rgb = RGB(0.2954, 0.4726, 0.2319)
            override fun previous(): FortuneColor {
                return GREEN
            }
        },

        YELLOW {
            // old values r: 0.315 g: 0.572 b: 0.112
            // correct values r: 0.3105 g: 0.5229 b: 0.1665
            override val rgb = RGB(0.2907, 0.4821, 0.2270)
        },

        BLUE {
            // old values r: 0.1 g: 0.422 b: 0.477
            // correct values r: 0.1872 g: 0.4568 b: 0.3559
            override val rgb = RGB(0.2685, 0.4841, 0.2470)
        },

        GREEN {
            // old values r: 0.143 g: 0.604 b: 0.251
            // correct values r: 0.2177 g: 0.5241 b: 0.2580
            override val rgb = RGB(0.2724, 0.4772, 0.2507)
            override fun next(): FortuneColor {
                return RED
            }
        };

        // RGB Values
        protected class RGB(var red: Double, var green: Double, var blue: Double)
        protected abstract val rgb: RGB

        val red get() = rgb.red
        val green get() = rgb.green
        val blue get() = rgb.blue

        protected open fun next(): FortuneColor {
            return values()[ordinal + 1]
        }

        protected open fun previous(): FortuneColor {
            return values()[ordinal - 1]
        }

        operator fun plus(increment: Int): FortuneColor {
            var times = increment
            var newColor = this
            while (times != 0) {
                newColor = newColor.next()
                times--
            }
            return newColor
        }

        operator fun minus(decrement: Int): FortuneColor {
            var times = decrement
            var newColor = this
            while (times != 0) {
                newColor = newColor.previous()
                times--
            }
            return newColor
        }

        // Find the position of a desired color relative to the current color
        fun findNearest(color: FortuneColor) =
                when (color) {
                    this + 1 -> 1
                    this - 1 -> -1
                    this + 2 -> 2
                    this -> 0
                    else -> throw IllegalStateException("${color.name} could not be found relative to ${this.name}")
                }

        companion object {
            // Fits a normal 8 bit color to a fortune color
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
