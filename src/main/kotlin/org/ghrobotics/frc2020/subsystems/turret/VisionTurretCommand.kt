/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.turret

import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.vision.GoalTracker
import org.ghrobotics.frc2020.vision.VisionProcessing
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.utils.toTransform

/**
 * A command that aligns the turret to the best available vision target.
 */
class VisionTurretCommand : FalconCommand(Turret) {
    override fun initialize() {
        // Turn on LEDs for tracking.
        VisionProcessing.turnOnLEDs()
    }

    override fun execute() {
        // Get the robot pose.
        val robotPose = Drivetrain.getPose()

        // Get the closest target.
        val target = GoalTracker.getClosestTarget(robotPose)

        if (target != null) {
            // Find the angle to the target.
            val angle = target.averagePose.relativeTo(robotPose + Turret.robotToTurret.toTransform()).rotation
            Turret.setAngle(Turret.angle + SIUnit(angle.radians))
        } else {
            // If there is no target, hold the current robot-relative angle.
            Turret.setAngle(Turret.angle)
        }
    }

    override fun end(interrupted: Boolean) {
        // Turn off LEDs for tracking.
        VisionProcessing.turnOffLEDs()
    }
}
