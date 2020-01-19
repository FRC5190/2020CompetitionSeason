/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.vision

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.milli
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
    private val pipelineEntry = subtable["pipeline"]
    private val latencyEntry = subtable["latency"]
    private val driverModeEntry = subtable["driver_mode"]
    private val isValidEntry = subtable["is_valid"]
    private val areaEntry = subtable["area"]
    private val poseEntry = subtable["pose"]

    /**
     * Returns whether the camera is connected or not.
     */
    val isConnected: Boolean
        get() = table.containsSubTable(cameraName)

    /**
     * Returns the vertical angle to the best target.
     */
    val pitch: Rotation2d
        get() = Rotation2d.fromDegrees(pitchEntry.getDouble(0.0))

    /**
     * Returns the horizontal angle to the best target.
     */
    val yaw: Rotation2d
        get() = Rotation2d.fromDegrees(-yawEntry.getDouble(0.0)) // Negating to make it CCW positive.

    /**
     * The area of the best target as a percentage of the total area of the screen.
     */
    val area: Double
        get() = areaEntry.getDouble(0.0)

    /**
     * The camera relative pose of the best target.
     */
    val pose: Pose2d?
        get() {
            val array = poseEntry.getDoubleArray(DoubleArray(0))
            return if (array.size == 3) {
                Pose2d(array[0], array[1], Rotation2d.fromDegrees(array[2]))
            } else null
        }

    /**
     * The transform that maps the camera pose to the target pose.
     */
    val transform: Transform2d?
        get() {
            val pose_ = pose ?: return null
            return Transform2d(pose_.translation, pose_.rotation)
        }

    /**
     * Represents the latency in the pipeline between the capture
     * and reception of data on the roboRIO.
     */
    val latency: SIUnit<Second>
        get() = latencyEntry.getDouble(0.0).milli.seconds

    /**
     * Returns whether a target exists and is valid.
     */
    val isValid: Boolean
        get() = isValidEntry.getBoolean(false)

    /**
     * Gets or sets the value of the pipeline. This can be used
     * to switch between different Vision pipelines.
     */
    var pipeline: Double
        get() = pipelineEntry.getDouble(0.0)
        set(value) {
            pipelineEntry.setDouble(value)
        }

    /**
     * Toggles "driver mode". In "driver mode", the camera stream will
     * not have any Vision processing artifacts displayed on it.
     */
    var driverMode: Boolean
        get() = driverModeEntry.getBoolean(false)
        set(value) {
            driverModeEntry.setBoolean(value)
        }
}
