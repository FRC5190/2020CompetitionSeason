/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.vision

import org.ghrobotics.lib.commands.FalconSubsystem

/**
 * Manages the Limelight on the robot.
 */
object LimelightManager : FalconSubsystem() {
    // Shooter Limelight
    private val limelight = Limelight()

    // Getters
    val isConnected get() = limelight.isConnected()
    val hasValidTarget get() = limelight.hasValidTarget
    val yaw get() = limelight.yaw
    val pitch get() = limelight.pitch
    val area get() = limelight.area
    val skew get() = limelight.skew
    val latency get() = limelight.latency

    fun turnOnLED() = limelight.turnOnLED()
    fun turnOffLED() = limelight.turnOffLED()
    fun blinkLED() = limelight.blinkLED()

    override fun periodic() {
        limelight.update()
    }
}
