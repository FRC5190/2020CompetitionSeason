package org.ghrobotics.frc2020.subsystems.fortunewheel

import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import org.ghrobotics.frc2020.FortuneWheelConstants
import kotlin.math.absoluteValue
import kotlin.math.floor
import kotlin.math.roundToInt

enum class FortuneColor {
    // Fortune Wheel colors
    RED { override val rgb = RGB(0.3559, 0.4287, 0.2153) },
    YELLOW { override val rgb = RGB(0.2927, 0.5378, 0.1694) },
    BLUE { override val rgb = RGB(0.1782, 0.4514, 0.3706) },
    GREEN { override val rgb = RGB(0.2136, 0.5288, 0.2575) },

    // Default color value
    BLACK { override val rgb = RGB(0.0, 0.0, 0.0) };

    // RGB Values
    protected class RGB(var red: Double, var green: Double, var blue: Double)
    protected abstract val rgb: RGB

    val red get() = rgb.red
    val green get() = rgb.green
    val blue get() = rgb.blue

    operator fun plus(increment: Int): FortuneColor {
        if (this == BLACK) { return BLACK
        }
        var number = ordinal + increment
        var color = number - (floor(number.toDouble()/4).toInt() * 4)
        return values()[color]
    }

    operator fun minus(decrement: Int): FortuneColor {
        if (this == BLACK) { return BLACK
        }
        var number = (ordinal - 4) - decrement
        var color = number + (floor(number.absoluteValue.toDouble()/4).toInt() * 4) + 4
        return values()[color]
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