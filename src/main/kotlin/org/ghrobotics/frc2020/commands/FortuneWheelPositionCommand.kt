/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import kotlin.math.roundToInt
import org.ghrobotics.frc2020.FortuneWheelConstants
import org.ghrobotics.frc2020.subsystems.FortuneWheelSpinner
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.operations.times

class FortuneWheelPositionCommand() : FalconCommand(FortuneWheelSpinner) {

    private var position = 0.meters

    constructor(position: SIUnit<Meter>) : this() {
        this.position = position
    }

    constructor(color: FortuneWheelSpinner.FortuneColor) : this() {
        var currentColor = FortuneWheelSpinner.sensorColor
        var testingColor = currentColor
        var index = 0
        while(testingColor != currentColor) {
            index++
            testingColor = testingColor.next()
        }

        this.position = FortuneWheelConstants.kContactColor * index
    }

    private val targetPosition = FortuneWheelConstants.kContactCirc * position

    // Status for LEDs
    val stageCount = (targetPosition / FortuneWheelConstants.kContactColor).value.roundToInt()
    var status = 0

    // Color Variables
    private var lastColor = FortuneWheelSpinner.FortuneColor.BLACK
    private var lastColorPosition: SIUnit<Meter> = 0.meters
    private var colorError: SIUnit<Meter> = 0.meters

    // Reset Encoder
    override fun initialize() {
        FortuneWheelSpinner.resetPosition()
    }

    override fun execute() {
        // Color Correction
        if (lastColor != FortuneWheelSpinner.sensorColor) {
            colorError += (FortuneWheelConstants.kContactColor).minus(FortuneWheelSpinner.spinnerPosition - lastColorPosition)
            lastColorPosition = FortuneWheelSpinner.spinnerPosition

            // Update Status
            status += 1
        }

        // Run Motor
        FortuneWheelSpinner.setVelocity((targetPosition.value - (FortuneWheelSpinner.spinnerPosition.value + colorError.value)).meters.velocity)

        lastColor = FortuneWheelSpinner.sensorColor
    }
}