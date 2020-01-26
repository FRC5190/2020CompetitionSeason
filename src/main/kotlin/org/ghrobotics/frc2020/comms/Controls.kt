/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.comms

import org.ghrobotics.frc2020.Robot
import org.ghrobotics.frc2020.TurretConstants
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.shooter.AutoShooterCommand
import org.ghrobotics.frc2020.subsystems.shooter.ManualShooterCommand
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.minutes
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.wrappers.hid.button
import org.ghrobotics.lib.wrappers.hid.kB
import org.ghrobotics.lib.wrappers.hid.kBumperRight
import org.ghrobotics.lib.wrappers.hid.kY
import org.ghrobotics.lib.wrappers.hid.xboxController

/**
 * Contains all the teleop controls for the robot.
 */
object Controls {
    val driverController = xboxController(0) {
        /**
         * Aim the turret and shoot power cells when the right bumper is pressed.
         * The turret and shooter will aim and the feeder will feed all balls
         * to the shooter when the drivetrain comes to a complete stop.
         */
        button(kBumperRight).change(Superstructure.aimTurret())

        /**
         * Jogs the turret zero a certain amount. This is useful when vision
         * align is consistently skewed to one side. Hitting the left
         * POV is useful when the turret is constantly aligning to the
         * right and vice versa.
         */
        pov(270).changeOn { Turret.jogZero(TurretConstants.kDefaultJogAmount) }
        pov(90).changeOn { Turret.jogZero(-TurretConstants.kDefaultJogAmount) }

        /**
         * Toggles climb mode. When climb mode is active, buttons which
         * allow operation of the climb subsystem are activated.
         */
        button(kB).changeOn { Robot.isClimbMode = !Robot.isClimbMode }

        /**
         * These are just buttons for debugging, will be removed for competition.
         */
        button(kY).change(AutoShooterCommand { 360.degrees / 1.minutes * 5000 })
        axisButton(5, 0.04) {
            change(ManualShooterCommand(source))
        }
    }

    fun update() {
        driverController.update()
    }
}
