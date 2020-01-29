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
import org.ghrobotics.frc2020.LEDConstants
import org.ghrobotics.lib.commands.FalconSubsystem

object LED : FalconSubsystem() {
    var led = AddressableLED(LEDConstants.kPort)
    var letBuffer: AddressableLEDBuffer = AddressableLEDBuffer(LEDConstants.kBufferSize)
    var rainbowFirstPixelHue = 0

    init {
        println("LED Subsystem init")
        led.setLength(letBuffer.length)
        reset()
        led.start()
    }

    fun reset() {
        println("LED Subsystem reset")
        setStatus(LEDStatus.RESET)
    }

    /*activates rainbow mode,
    should be used when all sensors are connected
    but when robot is disabled
     */
    fun rainbow() {
        println("LED Subsystem rainbow")

        for (i in 0..letBuffer.length) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            var hue = (rainbowFirstPixelHue + (i * 180 / letBuffer.length)) % 180
            // Set the value
            letBuffer.setHSV(i, hue, 255, 128)
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3
        // Check bounds
        rainbowFirstPixelHue %= 180
    }

    // Function for settings the colors of all RGB lights
    // You can manually input RGB values and a true or false statement for making the lights blink
    // THIS FUNCTION IS LIKELY TO CHANGE IN THE FUTURE, INCLUDING THE ARGUMENTS THE WILL BE PASSED TO IT
    // You can use a list of predefined enums to set colors, Enums are in the LEDStatus class within the same package
    // LEDSTATUS color enums default to no blinking, manually inputted colors blink by default
    fun setStatus(statusCode: LEDStatus.StatusColor) {
        println("LED Subsystem Set Status")

        if (!statusCode.blink) {
            for (i in 0 until letBuffer.length) { // Sets the specified LED to the RGB values for green
                letBuffer.setRGB(i, statusCode.r, statusCode.g, statusCode.b)
            }
            led.setData((letBuffer))
        } else {
            // will run code to flash led color
            // currently untested and may break when switching led modes
            // try rerunning the function with blink off if you encounter issues
            // involving changing the LED color after

//            while (statusCode.blink) {
                for (i in 0 until letBuffer.length) { // Sets the specified LED to the RGB values for green
                    letBuffer.setRGB(i, statusCode.r, statusCode.g, statusCode.b)
                }
//                Timer.delay(0.5)
//                for (i in 0 until letBuffer.length) { // Sets the specified LED to the RGB values for green
//                    letBuffer.setRGB(i, 0, 0, 0)
//                }
//                Timer.delay(0.5)
//            }
            led.setData(letBuffer)
        }
    }
}
