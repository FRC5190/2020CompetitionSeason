/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands.tests

import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.frc2020.subsystems.Shooter
import org.ghrobotics.frc2020.subsystems.SubsystemTestManager
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds

class TestShooterCommand : FalconCommand(Shooter) {

    private val timer = Timer()
    private var success = true

    override fun initialize() {
        // Start timer.
        timer.start()
    }

    override fun execute() {
        // Set shooter at 30% duty cycle and record encoder speed.
        Shooter.setPercent(0.3)

        // Check velocities when timer is greater than 1/4 a second.
        if (timer.get() > 0.25) {
            success = Shooter.velocity > 50.radians / 1.seconds
        }
    }

    override fun end(interrupted: Boolean) {
        timer.stop()
        Shooter.setNeutral()
        SubsystemTestManager.shooterCheck = success
    }
}
