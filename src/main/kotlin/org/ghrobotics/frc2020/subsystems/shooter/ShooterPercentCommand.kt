/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.shooter

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

/**
 * A command that allows manual control of the shooter.
 */
class ShooterPercentCommand(val source: DoubleSource) : FalconCommand(Shooter) {
    override fun execute() {
        println(source() * 12)
        Shooter.setPercent(source())
    }

    override fun end(interrupted: Boolean) {
        Shooter.setNeutral()
    }
}
