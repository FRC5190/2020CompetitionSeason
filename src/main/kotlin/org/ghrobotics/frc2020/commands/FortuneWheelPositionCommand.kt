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

class FortuneWheelPositionCommand(position: Double) : FalconCommand(FortuneWheelSpinner) {
    val targetPosition = FortuneWheelConstants.kContactCirc * position

    // Status for LEDs
    val stageCount = (targetPosition / FortuneWheelConstants.kContactColor).value.roundToInt()
    var status = 0

    // Create PID
    private var positionPID = PositionPID(targetPosition.value)

    // Color Variables
    private var lastColor = FortuneWheelSpinner.FortuneColor.Black
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

        // Execute PID
        FortuneWheelSpinner.setVelocity(positionPID.execute(FortuneWheelSpinner.spinnerPosition.value + colorError.value).meters.velocity)

        lastColor = FortuneWheelSpinner.sensorColor
    }

    private class PositionPID(val target: Double) {
        // PID Values
        val kP = FortuneWheelConstants.kFortuneWheelP
        val kI = FortuneWheelConstants.kFortuneWheelI
        val kD = FortuneWheelConstants.kFortuneWheelD

        // Variables
        var error = 0.0
//        var previousError = 0.0
//        var integral = 0.0
//        var derivative = 0.0
        var output = 0.0

        fun execute(value: Double): Double {
            // Calculate Output
            error = target - value
//            integral += (error*.02)
//            derivative = (error - previousError) / .02
//            previousError = error
            output = kP * error // + kI*integral + kD*derivative

            // Limit output to max RPM
            if (output > FortuneWheelConstants.kContactVelocity.value) {
                output = FortuneWheelConstants.kContactVelocity.value
            }
            // Return the result
            return output
        }
    }
}
