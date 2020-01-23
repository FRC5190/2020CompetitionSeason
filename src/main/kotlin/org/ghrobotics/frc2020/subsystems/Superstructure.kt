/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.frc2020.subsystems.turret.AutoTurretCommand
import org.ghrobotics.frc2020.subsystems.turret.VisionTurretCommand
import org.ghrobotics.frc2020.vision.GoalTracker
import org.ghrobotics.frc2020.vision.VisionProcessing
import org.ghrobotics.lib.commands.sequential

/**
 * Represents the overall superstructure of the robot, including the turret,
 * shooter, intake, and climbing mechanisms.
 */
object Superstructure {
    /**
     * Aims the turret at the goal.
     *
     * @param isAuto Whether we are in the autonomous period.
     * @return The command.
     */
    fun aimTurret(isAuto: Boolean = false): Command = sequential {
        // Turn on the LEDs so that we can start looking for the target.
        +InstantCommand(Runnable { VisionProcessing.turnOnLEDs() })

        // Turn the turret into the general area of the target, and cancel
        // when we see the target.
        if (isAuto) {
            TODO("Create field oriented command using odometry and angle to target")
        } else {
            +AutoTurretCommand.createFromFieldOrientedAngle(VisionConstants.kGoalFieldRelativeAngle)
                .withInterrupt(GoalTracker::isTrackingTargets)
        }

        // Make the turret track the goal position.
        +VisionTurretCommand()
    }.andThen(InstantCommand(Runnable { VisionProcessing.turnOffLEDs() }))
}
