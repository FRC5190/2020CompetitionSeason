/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.vision

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.milli
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.wrappers.networktables.FalconNetworkTable
import org.ghrobotics.lib.wrappers.networktables.get

/**
 * Represents a camera connected to a co-processor which is running
 * Chameleon Vision.
 */
class ChameleonCamera(private val cameraName: String) {
    // NetworkTable for the specific camera
    private val table = FalconNetworkTable.getTable("chameleon-vision")
    private val subtable: NetworkTable = table.getSubTable(cameraName)

    // Entries for the NetworkTable
    private val pitchEntry = subtable["pitch"]
    private val yawEntry = subtable["yaw"]
    private val latencyEntry = subtable["latency"]
    private val isValidEntry = subtable["is_valid"]
    private val poseEntry = subtable["pose"]

    private val periodicIO = PeriodicIO()

    /**
     * Returns whether the camera is connected or not.
     */
    val isConnected: Boolean
        get() = periodicIO.isConnected

    /**
     * Returns the vertical angle to the best target.
     */
    val pitch: Rotation2d
        get() = periodicIO.pitch

    /**
     * Returns the horizontal angle to the best target.
     */
    val yaw: Rotation2d
        get() = periodicIO.yaw

    /**
     * The transform that maps the camera pose to the target pose.
     */
    val transform: Transform2d?
        get() = periodicIO.transform

    /**
     * Represents the latency in the pipeline between the capture
     * and reception of data on the roboRIO.
     */
    val latency: SIUnit<Second>
        get() = periodicIO.latency

    /**
     * Returns whether a target exists and is valid.
     */
    val isValid: Boolean
        get() = periodicIO.isValid


    /**
     * Updates all values from NT.
     */
    fun update() {
        periodicIO.isConnected = table.containsSubTable(cameraName)
        if (periodicIO.isConnected) {
            periodicIO.pitch = Rotation2d.fromDegrees(pitchEntry.getDouble(0.0))
            periodicIO.yaw = Rotation2d.fromDegrees(-yawEntry.getDouble(0.0)) // Negating to make it CCW positive.

            val array = poseEntry.getDoubleArray(DoubleArray(0))
            periodicIO.transform = if (array.size == 3) {
                Transform2d(Translation2d(array[0], array[1]), Rotation2d.fromDegrees(array[2]))
            } else null

            periodicIO.latency = latencyEntry.getDouble(0.0).milli.seconds
            periodicIO.isValid = isValidEntry.getBoolean(false)
        }
    }

    private class PeriodicIO {
        var isConnected: Boolean = false
        var pitch: Rotation2d = Rotation2d()
        var yaw: Rotation2d = Rotation2d()

        var transform: Transform2d? = null
        var latency: SIUnit<Second> = 0.seconds
        var isValid: Boolean = false
    }
}
