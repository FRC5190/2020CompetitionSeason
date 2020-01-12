/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.comms

import org.ghrobotics.frc2020.commands.*
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.wrappers.hid.*

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
