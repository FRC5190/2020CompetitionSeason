/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.fortunewheel

import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import kotlin.math.absoluteValue
import kotlin.math.floor
import kotlin.math.roundToInt
import org.ghrobotics.frc2020.FortuneWheelConstants

enum class FortuneColor(val red: Double, val green: Double, val blue: Double) {
    // Default color value
    BLACK(0.0, 0.0, 0.0),

    // Fortune Wheel colors
    RED(0.3603,0.4282, 0.2116),
    YELLOW(0.2941, 0.5395, 0.1662),
    BLUE(0.17358, 0.45141, 0.375),
    GREEN(0.2087, 0.5371, 0.2539);

    operator fun plus(increment: Int): FortuneColor {
        if (this == BLACK) {
            return BLACK
        }
        var number = (ordinal + increment) % 4
        return values()[number]
    }

    operator fun minus(decrement: Int): FortuneColor {
        if (this == BLACK) {
            return BLACK
        }
        var number = (ordinal - decrement - 4) % 4 + 4
        return values()[number]
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
            var lrColor = Color8Bit(
                (color.red * resolution).roundToInt(),
                (color.green * resolution).roundToInt(),
                (color.blue * resolution).roundToInt()
            )

            for (fortune: FortuneColor in values()) {
                // Get a lower resolution version of the fortune color
                var lrFortune = Color8Bit(
                    (fortune.red * resolution).roundToInt(),
                    (fortune.green * resolution).roundToInt(),
                    (fortune.blue * resolution).roundToInt()
                )

                if (lrColor.red == lrFortune.red && lrColor.green == lrFortune.green && lrColor.blue == lrFortune.blue) {
                    return fortune
                }
            }
            return BLACK
        }
    }
}
