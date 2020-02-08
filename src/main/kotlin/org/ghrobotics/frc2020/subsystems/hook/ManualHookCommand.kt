/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.hook

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

/**
 * Moves the hook using a manual percent command.
 *
 * @param percentSource The percent command.
 */
class ManualHookCommand(private val percentSource: DoubleSource) : FalconCommand(Hook) {

    override fun execute() {
        Hook.setPercent(percentSource())
    }

    override fun end(interrupted: Boolean) {
        Hook.setNeutral()
    }
}
