/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.fortunewheel

import org.ghrobotics.frc2020.FortuneWheelConstants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.inInchesPerSecond
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.utils.Source

class FortuneWheelCommand() : FalconCommand(FortuneWheel) {

    private var status = Status.INITIALIZING

    private var targetColor = FortuneColor.BLACK
    private var targetColorSource = { FortuneColor.BLACK }
    private var targetCycles = 0
    private var targetDistance = 0.meters

    private var lastCompletion = -0.1

    private var accuracy = ColorAccuracy()

    constructor(cycles: Int) : this() {
        targetCycles = cycles
        targetDistance = FortuneWheelConstants.kColorDistance * cycles
    }

    constructor(color: FortuneColor) : this() {
        targetColorSource = { color }
    }

    constructor(colorSource: Source<FortuneColor>) : this() {
        targetColorSource = colorSource
    }

    init {
        status = Status.INITIALIZING
        FortuneWheel.resetPosition()
        println("")
        print("INITIALIZING > ")
    }

    override fun initialize() {
        targetColor = targetColorSource() + 2
        FortuneWheel.extendSpinnerPiston(true)
    }

    override fun execute() {
        when (status) {
            Status.INITIALIZING -> initializing()
            Status.RUNNING -> running()
            Status.CHECKING -> checking()
        }
    }

    fun initializing() {
        var currentColor = accuracy.add(FortuneWheel.sensorColor)
        if (currentColor != FortuneColor.BLACK) {
            print("color: $currentColor ")
            if (targetColor == FortuneColor.BLACK) {
                targetColor = currentColor + targetCycles
            } else {
                targetCycles = currentColor.findNearest(targetColor)
                targetDistance = FortuneWheelConstants.kColorDistance * targetCycles
            }

            print("target: $targetColor")

            FortuneWheel.resetPosition()
            println("")
            print("RUNNING > 0%")
            status = Status.RUNNING
        }
    }

    fun running() {
        FortuneWheel.setPosition(-targetDistance)

        var completion = FortuneWheel.spinnerPosition.value / targetDistance.value

        if (FortuneWheel.spinnerVelocity.inInchesPerSecond() < 1 && completion > 0.5) {
            print("CHECKING > $targetColor = ")
            status = Status.CHECKING
        }
    }

    fun checking() {
        var currentColor = accuracy.add(FortuneWheel.sensorColor)
        if (currentColor != FortuneColor.BLACK) {
            print("$currentColor? ... ")
            if (currentColor == targetColor) {
                println("Yes!")
                println("COMPLETE")
                FortuneWheel.setNeutral()
                status = Status.COMPLETE
            } else {
                println("No")
                print("CORRECTING > 0%")
                targetDistance += currentColor.findNearest(targetColor).inches * 2
                status = Status.RUNNING
            }
        }
    }

    override fun isFinished(): Boolean {
        return status == Status.COMPLETE
    }

    private enum class Status {
        INITIALIZING,
        RUNNING,
        CHECKING,
        COMPLETE
    }

    private class ColorAccuracy {
        private var colorMap: MutableMap<FortuneColor, Int> = mutableMapOf()

        fun add(color: FortuneColor): FortuneColor {
            colorMap.putIfAbsent(color, 0)
            colorMap.computeIfPresent(color) { _, u -> u + 1 }
            return if (colorMap[color]!! >= 30) {
                color
            } else {
                FortuneColor.BLACK
            }
        }

        fun clear() {
            colorMap.clear()
        }
    }
}
