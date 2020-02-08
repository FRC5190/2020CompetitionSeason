/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.forks

import org.ghrobotics.lib.commands.FalconCommand

class DropForksCommand(private val drop: Boolean) : FalconCommand(Forks) {

    override fun initialize() {
        Forks.dropForks(drop)
    }

    override fun isFinished(): Boolean {
        return true
    }
}
