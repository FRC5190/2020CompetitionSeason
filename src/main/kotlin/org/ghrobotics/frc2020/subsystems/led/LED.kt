/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.led

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.Color
import org.ghrobotics.frc2020.LEDConstants
import org.ghrobotics.frc2020.Robot
import org.ghrobotics.frc2020.subsystems.feeder.Feeder
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.inMilliseconds
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.wrappers.FalconTimedRobot

/**
 * Represents the addressable LEDs on the robot. These LEDs are
 * used to relay information to the driver and drive coach about
 * the status of various subsystems and the overall health of the
 * robot.
 */
object LED : FalconSubsystem() {

    // LED and its buffer.
    private val led = AddressableLED(LEDConstants.kPort)
    private val ledBuffer = AddressableLEDBuffer(LEDConstants.kBufferSize)

    // LED Colors
    private val kFault = Color.kRed
    private val kWaitingForCamera = Color.kLimeGreen
    private val kZeroingTurret = Color.kGreen
    private val kClimb = Color.kOrange

    // Current time
    private var currentTime = Timer.getFPGATimestamp().seconds

    // Variables to store status for rainbow mode and snake mode.
    private var rainbowFirstPixelHue = 0
    private var snakeFirstIndex = 0
    private var snakeMultiplier = 1

    // Constants for snake pattern
    private const val kNumLEDsInSnake = 5
    private const val kSnakeAdvancement = 1

    init {
        // Set the length from the LED buffer.
        led.setLength(ledBuffer.length)

        // Start streaming.
        led.start()
    }

    override fun periodic() {
        // Update current time.
        currentTime = Timer.getFPGATimestamp().seconds

        when {
            // Blink red when turret is not zeroed.
//            Robot.currentMode == FalconTimedRobot.Mode.DISABLED
//                && Turret.status == Turret.Status.NOT_ZEROED -> {
//                if (currentTime.inMilliseconds() % 1000 > 500) {
//                    setSolidColor(kFault)
//                } else {
//                    setSolidColor(Color.kBlack)
//                }
//            }
//
//            // Blink rapid green when turret is being zeroed.
//             Robot.currentMode == FalconTimedRobot.Mode.DISABLED
//                && Turret.status == Turret.Status.ZEROING -> {
//                if (currentTime.inMilliseconds() % 250 > 125) {
//                    setSolidColor(kZeroingTurret)
//                } else {
//                    setSolidColor(Color.kBlack)
//                }
//            }
//
//            // Snake pattern when waiting for camera.
//            !VisionProcessing.isConnected -> setSnakePattern(kWaitingForCamera)

            // Blink orange in climb mode.
            Robot.isClimbMode -> {
                if (currentTime.inMilliseconds() % 1000 > 500) {
                    setSolidColor(kClimb)
                } else {
                    setSolidColor(Color.kBlack)
                }
            }

            // Blue when intake sensor is triggered.
            Feeder.intakeSensorTriggered -> setSolidColor(Color.kBlue)

            // Rainbow when robot is disabled and everything is ready.
            Robot.currentMode == FalconTimedRobot.Mode.DISABLED -> setRainbow()

            // Off when none of the above conditions are met.
            else -> setSolidColor(Color.kBlack)
        }

        // Update with new data.
        led.setData(ledBuffer)
    }

    /**
     * Sets all LEDs in the LED strip to a solid color.
     *
     * @param color The color.
     */
    private fun setSolidColor(color: Color) {
        for (i in 0 until ledBuffer.length) {
            ledBuffer.setLED(i, color)
        }
    }

    /**
     * Sets a rainbow pattern on the LED buffer.
     */
    private fun setRainbow() {
        for (i in 0 until ledBuffer.length) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            val hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.length)) % 180
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128)
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3
        // Check bounds
        rainbowFirstPixelHue %= 180
    }

    /**
     * Sets a snake pattern on the LED buffer.
     *
     * @param color The color to use for the snake pattern.
     */
    private fun setSnakePattern(color: Color) {
        // Set multiplier.
        if (snakeFirstIndex == 0) {
            snakeMultiplier = 1
        } else if (snakeFirstIndex == ledBuffer.length - kNumLEDsInSnake) {
            snakeMultiplier = -1
        }

        // Make everything clear first.
        setSolidColor(Color.kBlack)

        // Set LEDs.
        for (i in 0 until kNumLEDsInSnake) {
            ledBuffer.setLED(snakeFirstIndex + i, color)
        }

        // Advance first index.
        snakeFirstIndex += kSnakeAdvancement * snakeMultiplier
    }
}
