/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.turret

import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.commands.FalconCommand

/**
 * A command that zeros the turret when the robot is being setup.
 */
class ZeroTurretCommand : FalconCommand(Turret) {
    private val timer = Timer()

    override fun initialize() = timer.start()

    override fun execute() {
        if (!Turret.hallEffectEngaged) timer.reset()
    }

    override fun end(interrupted: Boolean) {
        timer.stop()
        Turret.zero()
    }

    override fun isFinished() = timer.get() > 3.0
    override fun runsWhenDisabled() = true
}