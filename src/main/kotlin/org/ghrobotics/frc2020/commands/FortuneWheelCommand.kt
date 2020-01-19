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
import kotlin.math.pow

class FortuneWheelCommand() : FalconCommand(FortuneWheelSpinner) {

    private var colorTarget = FortuneWheelSpinner.FortuneColor.BLACK
    private var cycleTarget = 0
    private var cycle = 0
    private var direction = 0
    private var speed = 0.0
    private var correctCount = 0
    private var success = false

    // Rotate X number of cycles
    constructor(cycles: Int) : this() {
        colorTarget = FortuneWheelSpinner.sensorColor + cycles
        cycleTarget = cycles
    }

    // Rotate to color
    constructor(color: FortuneWheelSpinner.FortuneColor) : this() {
        var currentColor = FortuneWheelSpinner.sensorColor
        cycleTarget = currentColor.findNearest(color+2)
        colorTarget = color+2
    }

    // Save data for comparisons
    private var accuracy = Accuracy()

    // Prep data class
    override fun initialize() {
        accuracy.confirmed = FortuneWheelSpinner.sensorColor
        accuracy.lastConfirmed = FortuneWheelSpinner.sensorColor
    }

    override fun execute() {
        var currentColor = FortuneWheelSpinner.sensorColor
        if (accuracy.refresh(currentColor, direction)) { update() }
        direction = when{
            cycleTarget > cycle -> 1
            cycleTarget < cycle -> -1
            else -> 0
        }
    }

    fun update() {
        cycle = when{
            accuracy.confirmed == accuracy.lastConfirmed + 1 -> cycle + 1
            accuracy.confirmed == accuracy.lastConfirmed - 1 -> cycle - 1
            else -> cycle
        }

        speed = 1 - (2.0.pow(cycle.absoluteValue - cycleTarget.absoluteValue))

        println("    => " + accuracy.confirmed + " | $cycle -> $cycleTarget |")
        FortuneWheelSpinner.setPercent(-FortuneWheelConstants.kSpinnerSpeed * speed * direction)

        if (cycle == cycleTarget) {
            if (accuracy.confirmed != accuracy.lastConfirmed) {
                cycleTarget + accuracy.confirmed.findNearest(colorTarget)
            }else{
                correctCount++
            }
        } else {
            correctCount = 0
        }

        if (correctCount > FortuneWheelConstants.kCompletion) {
            FortuneWheelSpinner.setNeutral()
            success = true
        }
    }

    override fun isFinished(): Boolean {
        return success
    }

    private class Accuracy {
        private var weights = mutableMapOf<String, Int>()
        var confirmed = FortuneWheelSpinner.FortuneColor.BLACK
        var lastConfirmed = FortuneWheelSpinner.FortuneColor.BLACK

        // Update dataset and dump if max value reached
        fun refresh(color: FortuneWheelSpinner.FortuneColor, direction: Int): Boolean {
            weights.putIfAbsent(color.name, 0)
            weights.computeIfPresent(color.name) { key, value -> value + 1 }
            if (weights.getValue(color.name) == FortuneWheelConstants.kDataAccuracy && color != confirmed + 2 && color.name != "BLACK" && color != color - direction) {
                weights.clear()
                lastConfirmed = confirmed
                confirmed = color
                return true
            }
            return false
        }
    }
}
