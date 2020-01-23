/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.turret

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import org.ghrobotics.frc2020.TurretConstants
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.vision.GoalTracker
import org.ghrobotics.frc2020.vision.VisionProcessing
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit

/**
 * A command that aligns the turret to the best available vision target.
 */
class VisionTurretCommand : FalconCommand(Turret) {
    override fun initialize() {
        // Turn on LEDs for tracking.
        VisionProcessing.turnOnLEDs()
    }

    override fun execute() {
        // Get the predicted robot pose.
        val robotPose = Drivetrain.getPredictedPose(TurretConstants.kAlignDelay)

        // Get the turret pose assuming that the turret is locked to a robot-relative 0 degrees.
        val turretPose = robotPose + Transform2d(TurretConstants.kTurretRelativeToRobotCenter, Rotation2d())

        // Get the closest target.
        val target = GoalTracker.getClosestTarget(turretPose)

        if (target != null) {
            // Find the goal relative to the turret.
            val turretToGoal = target.averagePose.relativeTo(turretPose)

            // Find the angle to this goal.
            val angle = Rotation2d(turretToGoal.translation.x, turretToGoal.translation.y)

            // Set angle.
            Turret.setAngle(SIUnit(angle.radians))
        } else {
            // If there is no target, hold the current robot-relative angle.
            Turret.setAngle(Turret.getAngle())
        }
    }

    override fun end(interrupted: Boolean) {
        // Turn off LEDs for tracking.
        VisionProcessing.turnOffLEDs()
    }
}
