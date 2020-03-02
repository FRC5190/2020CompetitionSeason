/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.hood

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

/**
 * Allows for manual duty cycle control of the hood.
 */
class HoodPercentCommand(val percent: DoubleSource) : FalconCommand(Hood) {
    override fun execute() {
        Hood.setPercent(percent())
    }

    override fun end(interrupted: Boolean) {
        Hood.setNeutral()
    }
}
