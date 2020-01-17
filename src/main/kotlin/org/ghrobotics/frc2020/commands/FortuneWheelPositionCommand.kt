/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import org.ghrobotics.frc2020.FortuneWheelConstants
import org.ghrobotics.frc2020.subsystems.FortuneWheelSpinner
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.meters
import kotlin.math.absoluteValue

class FortuneWheelPositionCommand() : FalconCommand(FortuneWheelSpinner) {

    // Distance measured in cycles or number of color changes
    private var colorTarget = FortuneWheelSpinner.FortuneColor.BLACK
    private var cycleTarget = 0
    private var cycle = 0
    private var direction = 0

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

    // Prep data class
    override fun initialize() {
        accuracy.confirmed = FortuneWheelSpinner.sensorColor
        accuracy.lastConfirmed = FortuneWheelSpinner.sensorColor
        println("Target: $colorTarget")
}

    override fun execute() {
        var currentColor = FortuneWheelSpinner.sensorColor
        var direction = ((cycleTarget-cycle)/(cycleTarget-cycle).absoluteValue)
        if (accuracy.refresh(currentColor, direction)) { update() }
    }

    fun update() {
        if (accuracy.confirmed != accuracy.lastConfirmed) {
            if (accuracy.lastConfirmed + 1 == accuracy.confirmed) {
                cycle++
            }
            if (accuracy.lastConfirmed - 1 == accuracy.confirmed) {
                cycle--
            }
        }

        //println("")
        //println("    =>  " + accuracy.confirmed.toString() + " | $cycle -> $cycleTarget")

        FortuneWheelSpinner.setPercent(direction * 0.2)

        if (cycle == cycleTarget) {
            if (accuracy.confirmed != accuracy.lastConfirmed) {
                cycleTarget + accuracy.confirmed.findNearest(colorTarget)
            } else {
                this.end(false)
            }
        }
    }

    private class Accuracy {
        private var weights = mutableMapOf<String, Int>()
        var confirmed = FortuneWheelSpinner.FortuneColor.BLACK
        var lastConfirmed = FortuneWheelSpinner.FortuneColor.BLACK

        // Update dataset and dump if max value reached
        fun refresh(color: FortuneWheelSpinner.FortuneColor, direction: Int): Boolean {
//            print( when(color.name){
//                "BLACK" -> "*"
//                "RED" -> "r"
//                "YELLOW" -> "y"
//                "BLUE" -> "b"
//                "GREEN" -> "g"
//                else -> " "
//            })
            weights.putIfAbsent(color.name, 0)
            weights.computeIfPresent(color.name) { key, value -> value + 1 }
            if (weights.getValue(color.name) == FortuneWheelConstants.kDataAccuracy && (color.name == confirmed.name || color.name == (confirmed + direction).name) ) {
                weights.clear()
                lastConfirmed = confirmed
                confirmed = color
                return true
            }
            return false
        }
    }
}
