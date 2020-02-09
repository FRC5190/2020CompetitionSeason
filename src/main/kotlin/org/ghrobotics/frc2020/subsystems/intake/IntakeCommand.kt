/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.intake

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

class IntakeCommand(private val percentSource: DoubleSource, private val percentSource2: DoubleSource = { 0.0 }) :
    FalconCommand(Intake) {

    override fun initialize() {
        Intake.extendPiston(true)
    }

    override fun execute() {
        Intake.setPercent(percentSource(), percentSource2())
    }

    override fun end(interrupted: Boolean) {
        Intake.extendPiston(false)
        Intake.setNeutral()
    }
}
