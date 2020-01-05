/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2020.Controls
import org.ghrobotics.frc2020.subsystems.Drivetrain
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kA

/**
 * Command to drive the robot using the Xbox controller in teleop.
 */
class TeleopDriveCommand : FalconCommand(Drivetrain) {
    override fun execute() {
        Drivetrain.curvatureDrive(-xSource(), cSource(), qSource())
    }

    companion object {
        val xSource = Controls.driverController.getY(GenericHID.Hand.kLeft)
        val cSource = Controls.driverController.getX(GenericHID.Hand.kLeft)
        val qSource = Controls.driverController.getRawButton(kA)
    }
}
