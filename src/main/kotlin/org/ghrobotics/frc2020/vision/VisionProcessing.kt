/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.vision

import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.frc2020.kIsRaceRobot
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.utils.InterpolatingTreeMap
import org.ghrobotics.lib.utils.toTransform

/**
 * Object that handles Vision Processing on the robot.
 */
object VisionProcessing : FalconSubsystem() {

    // The Green LED to track targets.
    private val led = DigitalOutput(VisionConstants.kLEDId)

    // The camera on the turret.
    private val camera = ChameleonCamera("USB Camera-B4.09.24.1")

    // PeriodicIO.
    private val periodicIO = PeriodicIO()

    // Lookup table for pitch to distance
    private val lookupTable = InterpolatingTreeMap.createFromSI<Radian, Meter>()

    init {
        if (kIsRaceRobot) {
            lookupTable[13.90.degrees] = 7.feet
            lookupTable[10.10.degrees] = 8.feet
            lookupTable[06.84.degrees] = 9.feet
            lookupTable[04.17.degrees] = 10.feet
            lookupTable[02.26.degrees] = 11.feet
            lookupTable[00.60.degrees] = 12.feet
            lookupTable[(-0.95).degrees] = 13.feet
            lookupTable[(-2.96).degrees] = 14.feet
            lookupTable[(-3.51).degrees] = 15.feet
            lookupTable[(-4.35).degrees] = 16.feet
            lookupTable[(-7.31).degrees] = 20.feet
            lookupTable[(-10.17).degrees] = 25.feet
            lookupTable[(-11.45).degrees] = 28.feet
        } else {
            lookupTable[19.30.degrees] = 8.feet
            lookupTable[14.67.degrees] = 9.feet
            lookupTable[12.60.degrees] = 10.feet
            lookupTable[08.99.degrees] = 11.feet
            lookupTable[06.31.degrees] = 12.feet
            lookupTable[03.24.degrees] = 13.feet
            lookupTable[02.50.degrees] = 14.feet
            lookupTable[00.76.degrees] = 15.feet
            lookupTable[00.15.degrees] = 16.feet
            lookupTable[(-4.45).degrees] = 20.feet
            lookupTable[(-7.70).degrees] = 24.feet
            lookupTable[(-8.67).degrees] = 26.feet
            lookupTable[(-9.52).degrees] = 28.feet
        }
    }

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
     * Returns whether the camera is connected.
     */
    val isConnected get() = camera.isConnected

    /**
     * Returns the distance along the ground to the goal.
     *
     * @return The distance along the ground to the goal.
     */
    val distance: SIUnit<Meter>
        get() = lookupTable[SIUnit(camera.pitch.radians)]!!

    var estimatedRobotPose: Pose2d = Pose2d()
        private set

    override fun periodic() {
        // Update camera.
        camera.update()

        // Substitute (albeit very accurate) for solvePnP until solvePnP is fixed.
        val cameraToTarget = Transform2d(Translation2d(distance * angle.cos, distance * angle.sin), Rotation2d())

        // Add solvePnP pose to GoalTracker.
        if (cameraToTarget != Transform2d() && camera.isValid) {
            val latency = camera.latency
            val timestamp = Timer.getFPGATimestamp().seconds - latency

            val turretToTarget = VisionConstants.kTurretToCamera + cameraToTarget
            val robotToTarget = Turret.getRobotToTurret(timestamp) + turretToTarget.toTransform()
            val fieldRelativeTarget = Drivetrain.getPose(timestamp) + robotToTarget.toTransform()

            GoalTracker.addSample(timestamp, fieldRelativeTarget)
            estimatedRobotPose =
                GoalLocalizer.calculateRobotPose(timestamp, cameraToTarget, Drivetrain.getPose().rotation)
        }

        // Update GoalTracker.
        GoalTracker.update()

        led.set(!periodicIO.desiredLEDState)
    }

    /**
     * Turns on the LEDs for vision tracking.
     */
    fun turnOnLEDs() {
        periodicIO.desiredLEDState = true
    }

    /**T
     * Turns off the LEDs for vision tracking.
     */
    fun turnOffLEDs() {
        periodicIO.desiredLEDState = false
    }

    private class PeriodicIO {
        var desiredLEDState: Boolean = false
    }
}
