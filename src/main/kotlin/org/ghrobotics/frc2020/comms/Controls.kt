/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.comms

import org.ghrobotics.frc2020.subsystems.shooter.AutoShooterCommand
import org.ghrobotics.frc2020.subsystems.turret.AutoTurretCommand
import org.ghrobotics.frc2020.subsystems.turret.ManualTurretCommand
import org.ghrobotics.frc2020.subsystems.turret.VisionTurretCommand
import org.ghrobotics.frc2020.subsystems.turret.ZeroTurretCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.degrees
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
        button(kB).change(VisionTurretCommand())
        button(kY).change(AutoShooterCommand { SIUnit(615.0) })

        axisButton(5, 0.04) {
            change(ManualTurretCommand(source))
        }
    }

    fun update() {
        driverController.update()
    }
}
