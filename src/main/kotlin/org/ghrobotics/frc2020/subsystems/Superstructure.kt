/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.ghrobotics.frc2020.TurretConstants
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.turret.AutoTurretCommand
import org.ghrobotics.frc2020.vision.GoalTracker
import org.ghrobotics.frc2020.vision.VisionProcessing
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit

/**
 * Represents the overall superstructure of the robot, including the turret,
 * shooter, intake, and climbing mechanisms.
 */
object Superstructure {
    // Latest aiming parameters for the superstructure.
    private var latestAimingParameters = AimingParameters(Rotation2d(), SIUnit(0.0))

    /**
     * Aims the turret at the goal.
     *
     * @return The command.
     */
    fun aimTurret(): Command = sequential {
        // Turn on the LEDs so that we can start looking for the target.
        +InstantCommand(Runnable { VisionProcessing.turnOnLEDs() })
        // Add a slight delay so that we can register the target.
        +WaitCommand(0.2)
        // Turn the turret.
        +AutoTurretCommand { SIUnit(latestAimingParameters.turretAngle.radians) }
        // Turn off LEDs.
    }.andThen(InstantCommand(Runnable { VisionProcessing.turnOffLEDs() }))

    /**
     * Returns the latest aiming parameters for the shooter and the turret.
     *
     * @return The latest aiming parameters for the shooter and the turret.
     */
    private fun getAimingParameters(): AimingParameters {
        // Get the predicted field-relative robot pose.
        val robotPose = Drivetrain.getPredictedPose(TurretConstants.kAlignDelay)

        // Get the turret pose, assuming the turret is locked to 0 degrees.
        val turretPose = robotPose + Transform2d(TurretConstants.kTurretRelativeToRobotCenter, Rotation2d())

        // Get the target that is closest to the turret pose.
        val target = GoalTracker.getClosestTarget(turretPose)

        val turretToGoal = if (target != null) {
            // Get the goal's pose in turret's coordinates.
            target.averagePose.relativeTo(turretPose)
        } else {
            // Get an approximation from the current robot pose and the known target pose.
            VisionConstants.kGoalLocation.relativeTo(turretPose)
        }

        // Get the distance to the target.
        val distance = turretPose.translation.norm

        // Get the angle to the target.
        val angle = Rotation2d(turretToGoal.translation.x, turretToGoal.translation.y)

        // Return the distance and angle.
        return AimingParameters(angle, SIUnit(distance))
    }

    /**
     * Updates all superstructure states.
     */
    fun update() {
        latestAimingParameters = getAimingParameters()
    }

    /**
     * Aiming parameters for the shooter and the turret.
     */
    data class AimingParameters(
        val turretAngle: Rotation2d,
        val distance: SIUnit<Meter>
    )
}
