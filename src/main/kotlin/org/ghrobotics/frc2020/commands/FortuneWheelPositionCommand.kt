/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import org.ejml.equation.IntegerSequence
import org.ghrobotics.frc2020.FortuneWheelConstants
import org.ghrobotics.frc2020.subsystems.FortuneWheelSpinner
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.velocity

class FortuneWheelPositionCommand() : FalconCommand(FortuneWheelSpinner) {

    // Distance measured in cycles or number of color changes
    private var colorTarget = FortuneWheelSpinner.FortuneColor.BLACK
    private var cycleTarget = 0
    private var cycle = 0

    // Rotate X number of cycles
    constructor(cycles: Int) : this() {
        colorTarget = FortuneWheelSpinner.sensorColor + cycles
        cycleTarget = cycles
    }

    // Rotate to color
    constructor(color: FortuneWheelSpinner.FortuneColor) : this() {
        var currentColor = FortuneWheelSpinner.sensorColor
        cycleTarget = currentColor.findNearest(color)
        colorTarget = color
    }

    // Save data for comparisons
    private var accuracy = Accuracy()
    var lastColor = FortuneWheelSpinner.FortuneColor.BLACK

    // Prep data class
    override fun initialize() {
        accuracy.lastConfirmed = FortuneWheelSpinner.sensorColor
    }

    override fun execute() {
        var currentColor = FortuneWheelSpinner.sensorColor
        accuracy.update(currentColor)

        // Run motor if target has not been reached
        if (cycle != cycleTarget){
            var velocity = (FortuneWheelConstants.kContactColor * (cycleTarget - cycle)).velocity
            if (velocity > FortuneWheelConstants.kContactVelocity) velocity = FortuneWheelConstants.kContactVelocity else velocity
            if (velocity < -FortuneWheelConstants.kContactVelocity) velocity = -FortuneWheelConstants.kContactVelocity else velocity
            FortuneWheelSpinner.setVelocity(velocity)
        } else {
            // If target was reached, but it is wrong color, find correct color
            if (currentColor != colorTarget) {
                colorTarget + currentColor.findNearest(colorTarget)
            }else{
                if (currentColor == accuracy.lastConfirmed) {
                    this.end(false)
                }
            }
        }

        // Check if color changed to next color
        if (lastColor != currentColor) {
            println(currentColor.name)
            if (lastColor == accuracy.lastConfirmed) {
                if (lastColor + 1 == currentColor) {
                    cycle++
                }
                if (lastColor - 1 == currentColor) {
                    cycle--
                }
            }
        }
    }

    private class Accuracy {
        private var weights = mutableMapOf<FortuneWheelSpinner.FortuneColor, Int>()
        var lastConfirmed = FortuneWheelSpinner.FortuneColor.BLACK

        // Update dataset and dump if max value reached
        fun update(color: FortuneWheelSpinner.FortuneColor) {
            weights.computeIfPresent(color) { key, value -> value + 1 }
            if (weights.getValue(color) == FortuneWheelConstants.kDataAccuracy && color != FortuneWheelSpinner.FortuneColor.BLACK) {
                weights.clear()
                lastConfirmed = color
            }
        }
    }
}