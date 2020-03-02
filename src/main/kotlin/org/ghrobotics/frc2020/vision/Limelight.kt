/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.vision

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.milli
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.wrappers.networktables.get

/**
 * Wrapper class for the Limelight camera.
 */
class Limelight(name: String = "limelight") {

    // Constants
    private val kImageCaptureLatency = 11.milli.seconds

    // Table which contains entries.
    private val table: NetworkTable = NetworkTableInstance.getDefault().getTable(name)

    // Target data
    private val tv = table["tv"]
    private val tx = table["tx"]
    private val ty = table["ty"]
    private val ta = table["ta"]
    private val ts = table["ts"]
    private val tl = table["tl"]

    // Camera controls
    private val ledMode = table["ledMode"]
    private val camMode = table["camMode"]
    private val pipeline = table["pipeline"]

    // PeriodicIO.
    private val periodicIO = PeriodicIO()

    // Getters
    val hasValidTarget get() = periodicIO.hasValidTarget
    val yaw get() = periodicIO.yaw
    val pitch get() = periodicIO.pitch
    val area get() = periodicIO.area
    val skew get() = periodicIO.skew
    val latency get() = periodicIO.latency

    /**
     * Turns on the LED.
     */
    fun turnOnLED() = ledMode.setNumber(3)

    /**
     * Turns off the LED.
     */
    fun turnOffLED() = ledMode.setNumber(1)

    /**
     * Blinks the LED.
     */
    fun blinkLED() = ledMode.setNumber(2)

    /**
     * Sets the currently selected pipeline.
     */
    fun setPipeline(id: Int) = pipeline.setNumber(id)

    /**
     * Switches between processing and driver mode.
     */
    fun setDriverMode(driverMode: Boolean = true) = camMode.setNumber(if (driverMode) 1 else 0)

    /**
     * Returns whether the Limelight is connected.
     */
    fun isConnected() = periodicIO.latency > kImageCaptureLatency

    /**
     * Updates the camera with new information.
     */
    fun update() {
        with(periodicIO) {
            hasValidTarget = tv.getDouble(0.0) == 1.0

            yaw = -tx.getDouble(0.0).degrees // negating because LL returns CW positive
            pitch = ty.getDouble(0.0).degrees

            area = ta.getDouble(0.0)
            skew = ts.getDouble(0.0).degrees

            latency = tl.getDouble(0.0).milli.seconds + kImageCaptureLatency
        }
    }

    /**
     * Stores data about the target.
     */
    private class PeriodicIO {
        var hasValidTarget: Boolean = false

        var yaw: SIUnit<Radian> = 0.radians
        var pitch: SIUnit<Radian> = 0.radians

        var area: Double = 0.0
        var skew: SIUnit<Radian> = 0.radians

        var latency: SIUnit<Second> = (-1).seconds
    }
}
