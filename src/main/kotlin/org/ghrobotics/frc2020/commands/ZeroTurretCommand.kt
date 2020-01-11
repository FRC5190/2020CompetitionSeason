/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.frc2020.subsystems.Turret
import org.ghrobotics.lib.commands.FalconCommand

class ZeroTurretCommand : FalconCommand(Turret) {

    private val timer = Timer()

    override fun initialize() {
        timer.start()
    }

    override fun execute() {
        if (!Turret.hallEffectEngaged) {
            // timer.reset() // For testing with 775Pro.
        }
    }

    override fun end(interrupted: Boolean) {
        timer.stop()
        Turret.zero()
    }

    override fun isFinished(): Boolean {
        // return timer.get() > 3.0
        return true // For testing with 775Pro.
    }

    override fun runsWhenDisabled(): Boolean {
        return true
    }
}
