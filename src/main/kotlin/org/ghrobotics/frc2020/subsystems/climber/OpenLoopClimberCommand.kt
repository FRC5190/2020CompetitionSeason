/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.climber

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

class OpenLoopClimberCommand(private val percentSource: DoubleSource) : FalconCommand(Climber) {

    override fun execute() {
        Climber.setBrake(false)
        Climber.setPercent(percentSource())
    }

    override fun end(interrupted: Boolean) {
        Climber.setBrake(true)
        Climber.setNeutral()
    }
}
