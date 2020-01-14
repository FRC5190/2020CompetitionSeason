/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.comms

import org.ghrobotics.frc2020.commands.ManualShooterCommand
import org.ghrobotics.frc2020.commands.ManualTurretCommand
import org.ghrobotics.frc2020.commands.VisionTurretCommand
import org.ghrobotics.frc2020.commands.ZeroTurretCommand
import org.ghrobotics.lib.wrappers.hid.button
import org.ghrobotics.lib.wrappers.hid.kA
import org.ghrobotics.lib.wrappers.hid.kB
import org.ghrobotics.lib.wrappers.hid.kY
import org.ghrobotics.lib.wrappers.hid.xboxController

/**
 * Contains all the teleop controls for the robot.
 */
object Controls {
    val driverController = xboxController(0) {
        button(kA).change(ZeroTurretCommand())
        button(kB).change(ManualTurretCommand { 0.3 })
        button(kY).change(VisionTurretCommand())

        axisButton(5, 0.04) {
            change(ManualShooterCommand(source))
        }
    }

    fun update() {
        driverController.update()
    }
}
