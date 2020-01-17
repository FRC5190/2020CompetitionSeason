/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.vision

import edu.wpi.first.wpilibj.DigitalOutput
import kotlin.math.tan
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit

/**
 * Object that handles Vision Processing on the robot.
 */
object VisionProcessing {
    // The Green LED to track targets.
    private val led = DigitalOutput(VisionConstants.kLEDId)

    // The camera on the turret.
    private val camera = ChameleonCamera("USB Camera-B4.09.24.1")

    /**
     * Returns the angle to the best target.
     *
     * @return The angle to the best target.
     */
    val angle get() = camera.yaw

    /**
     * Returns whether the camera sees a valid target.
     */
    val isValid get() = camera.isValid

    /**
     * Returns the distance along the ground to the goal.
     *
     * @return The distance along the ground to the goal.
     */
    val distance: SIUnit<Meter>
        get() {
            val deltaHeight = VisionConstants.kGoalHeight - VisionConstants.kCameraHeight
            return deltaHeight / tan(camera.pitch.radians + VisionConstants.kCameraAngle.radians)
        }

    /**
     * Turns on the LEDs for vision tracking.
     */
    fun turnOnLEDs() {
        led.set(false)
    }

    /**
     * Turns off the LEDs for vision tracking.
     */
    fun turnOffLEDs() {
        led.set(true)
    }
}
