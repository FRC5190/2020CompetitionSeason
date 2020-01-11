/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import org.ghrobotics.frc2020.StatusColor
import org.ghrobotics.frc2020.commands.LEDCommand
import org.ghrobotics.lib.commands.FalconSubsystem

object LEDSubsystem : FalconSubsystem() {
    var m_led = AddressableLED(9)
    var m_ledBuffer: AddressableLEDBuffer = AddressableLEDBuffer(60)
    var m_rainbowFirstPixelHue = 0

    init {
        defaultCommand = LEDCommand()
        m_led.setLength(m_ledBuffer.getLength())
        reset()
        m_led.start()
        println("LED Subsystem init")
    }

    fun initializing(): Boolean {
        println("LED Subsystem initializing")
        setStatus(StatusColor(254, 127, 156))
        return false
    }

    fun ready(): Boolean {
        println("LED Subsystem ready")
        setStatus(StatusColor(0, 128, 0))
        return true
    }

    override fun periodic() {
        super.periodic()
        println("LED Subsystem periodic")
    }

    fun reset() {
        println("LED Subsystem reset")
        setStatus(StatusColor(0, 0, 0))
    }

    fun rainbow() {
        println("LED Subsystem rainbow")

        for (i in 0..m_ledBuffer.length) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128)
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3
        // Check bounds
        m_rainbowFirstPixelHue %= 180
    }

    fun setStatus(statusCode: StatusColor) {
        println("LED Subsystem setstatus")

        for (i in 0 until m_ledBuffer.length) { // Sets the specified LED to the RGB values for green
            m_ledBuffer.setRGB(i, statusCode.r, statusCode.g, statusCode.b)
        }
        m_led.setData(m_ledBuffer)
    }
}
