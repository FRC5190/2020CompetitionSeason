/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.drivetrain

import edu.wpi.first.wpilibj.GenericHID
import kotlin.math.abs
import org.ghrobotics.frc2020.comms.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kX

/**
 * Command to drive the robot using the Xbox controller in teleop.
 */
class DrivetrainTeleopCommand : FalconCommand(Drivetrain) {

    override fun execute() {
        Drivetrain.curvatureDrive(-xSource() * abs(xSource()), cSource(), qSource())
    }

    companion object {
        val xSource = Controls.driverController.getY(GenericHID.Hand.kLeft).withDeadband(0.05)
        val cSource = Controls.driverController.getX(GenericHID.Hand.kLeft).withDeadband(0.05)
        val qSource = Controls.driverController.getRawButton(kX)
    }
}
