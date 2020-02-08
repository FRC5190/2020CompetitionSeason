/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.climber

import org.ghrobotics.lib.commands.FalconCommand

/**
 * Extends or retracts the climber assembly.
 *
 * @param extend Whether to extend or retract the climber assembly.
 */
class ExtendClimberCommand(private val extend: Boolean) : FalconCommand(Climber) {
    override fun initialize() {
        Climber.extend(extend)
    }

    override fun isFinished(): Boolean {
        return true
    }
}
