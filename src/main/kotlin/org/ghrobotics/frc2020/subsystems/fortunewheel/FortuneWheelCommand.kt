/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.fortunewheel

import kotlin.math.absoluteValue
import kotlin.math.pow
import org.ghrobotics.frc2020.FortuneWheelConstants
import org.ghrobotics.lib.commands.FalconCommand

class FortuneWheelCommand() : FalconCommand(FortuneWheel) {

    private var colorTarget = FortuneColor.BLACK
    private var cycleTarget = 0
    private var cycle = 0
    private var direction = 0
    private var speed = 0.0
    private var correctCount = 0
    private var success = false

    // Rotate X number of cycles
    constructor(cycles: Int) : this() {
        colorTarget = FortuneWheel.sensorColor + cycles
        cycleTarget = cycles
    }

    // Rotate to color
    constructor(color: FortuneColor) : this() {
        var currentColor = FortuneWheel.sensorColor
        cycleTarget = currentColor.findNearest(color + 2)
        colorTarget = color + 2
    }

    // Save data for comparisons
    private var accuracy = Accuracy()

    // Prep data class
    override fun initialize() {
        accuracy.confirmed = FortuneWheel.sensorColor
        accuracy.lastConfirmed = FortuneWheel.sensorColor
    }

    override fun execute() {
        var currentColor = FortuneWheel.sensorColor
        if (accuracy.refresh(currentColor, direction)) { update() }
        direction = when {
            cycleTarget > cycle -> 1
            cycleTarget < cycle -> -1
            else -> 0
        }
    }

    fun update() {
        cycle = when {
            accuracy.confirmed == accuracy.lastConfirmed + 1 -> cycle + 1
            accuracy.confirmed == accuracy.lastConfirmed - 1 -> cycle - 1
            else -> cycle
        }

        speed = 1 - (2.0.pow(cycle.absoluteValue - cycleTarget.absoluteValue))

        println("    => " + accuracy.confirmed + " | $cycle -> $cycleTarget |")
        FortuneWheel.setPercent(-FortuneWheelConstants.kSpinnerSpeed * speed * direction)

        if (cycle == cycleTarget) {
            if (accuracy.confirmed != accuracy.lastConfirmed) {
                cycleTarget + accuracy.confirmed.findNearest(colorTarget)
            } else {
                correctCount++
            }
        } else {
            correctCount = 0
        }

        if (correctCount > FortuneWheelConstants.kCompletion) {
            FortuneWheel.setNeutral()
            success = true
        }
    }

    override fun isFinished(): Boolean {
        return success
    }

    private class Accuracy {
        private var weights = mutableMapOf<String, Int>()
        var confirmed = FortuneColor.BLACK
        var lastConfirmed = FortuneColor.BLACK

        // Update dataset and dump if max value reached
        fun refresh(color: FortuneColor, direction: Int): Boolean {
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
