/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.hood

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.utils.Source

/**
 * Sets the hood to a specific angle.
 */
class AutoHoodCommand(private val angle: Source<SIUnit<Radian>>) : FalconCommand(Hood) {
    override fun execute() {
        Hood.setAngle(angle())
    }

    override fun end(interrupted: Boolean) {
        Hood.setNeutral()
    }
}
