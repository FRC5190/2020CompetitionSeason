/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import edu.wpi.first.wpilibj.geometry.Pose2d
import org.ghrobotics.frc2020.subsystems.Drivetrain
import org.ghrobotics.frc2020.subsystems.SubsystemTestManager
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.inches

/**
 * Runs a series of tests on the Drivetrain.
 */
class TestDrivetrainCommand : FalconCommand(Drivetrain) {

    // The test state.
    private var success = true

    override fun initialize() {
        // Reset drivetrain encoders.
        Drivetrain.resetPosition(Pose2d())
    }

    override fun execute() {
        // Make the robot drive straight using open loop control.
        Drivetrain.setPercent(0.2, 0.2)
    }

    override fun end(interrupted: Boolean) {
        // Make sure the encoders have traveled at least 3 inches in the positive direction.
        success = Drivetrain.leftPosition > 3.inches && Drivetrain.rightPosition > 3.inches
        Drivetrain.setNeutral()
        SubsystemTestManager.drivetrainCheck = success
    }
}
