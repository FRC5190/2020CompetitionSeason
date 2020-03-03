/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.vision

import edu.wpi.first.wpilibj.geometry.Transform2d
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.inches
import kotlin.math.tan

/**
 * Manages the Limelight on the robot.
 */
object LimelightManager : FalconSubsystem() {

    // Constants
    private val kCameraHeight = 24.5.inches
    private val kGoalHeight = 89.5.inches
    private val kCameraAngle = 35.degrees

    // Shooter Limelight
    private val turretLimelight = Limelight()

    /**
     * Returns whether the turret limelight is connected.
     */
    fun isTurretLimelightConnected() = turretLimelight.isConnected()

    /**
     * Returns whether the turret Limelight has a valid target.
     */
    fun doesTurretLimelightHaveValidTarget() = turretLimelight.hasValidTarget

    /**
     * Returns the distance to the target.
     */
    fun getDistanceToGoal(): SIUnit<Meter> {
        return (kGoalHeight - kCameraHeight) / tan((turretLimelight.pitch + kCameraAngle).value)
    }

    /**
     * Returns the angle to the target.
     */
    fun getAngleToGoal(): SIUnit<Radian> = turretLimelight.yaw

    /**
     * Turns on the Limelight LED.
     */
    fun turnOnLED() = turretLimelight.turnOnLED()

    /**
     * Turns off the Limelight LED.
     */
    fun turnOffLED() = turretLimelight.turnOffLED()

    /**
     * Blinks the Limelight LED.
     */
    fun blinkLED() = turretLimelight.blinkLED()

    /**
     * Updates the Limelight.
     */
    override fun periodic() {
        turretLimelight.update()
    }
}
