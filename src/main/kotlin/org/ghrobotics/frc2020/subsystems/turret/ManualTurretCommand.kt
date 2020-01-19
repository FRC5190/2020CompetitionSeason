/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.turret

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

/**
 * A command that can be used for manual control of the turret.
 */
class ManualTurretCommand(val percent: DoubleSource) : FalconCommand(Turret) {
    override fun execute() = Turret.setPercent(percent())
    override fun end(interrupted: Boolean) = Turret.setPercent(0.0)
}
