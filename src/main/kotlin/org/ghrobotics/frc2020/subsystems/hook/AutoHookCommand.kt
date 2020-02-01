/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.hook

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit

class AutoHookCommand(private val desiredPosition: SIUnit<Meter>) : FalconCommand(Hook) {

    override fun execute() {
        Hook.setPosition(desiredPosition)
    }

    override fun end(interrupted: Boolean) {
        Hook.setNeutral()
    }
}
