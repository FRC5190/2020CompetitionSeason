/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.turret

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.Source

class TurretPercentCommand(private val percent: Source<Double>) : FalconCommand(Turret) {
    override fun execute() {
        Turret.setPercent(percent())
    }

    override fun end(interrupted: Boolean) {
        Turret.setNeutral()
    }
}
