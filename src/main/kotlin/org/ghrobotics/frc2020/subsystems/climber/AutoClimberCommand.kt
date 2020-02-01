/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.climber

import org.ghrobotics.frc2020.subsystems.hook.AutoHookCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit

class AutoClimberCommand(private val desiredHeight: SIUnit<Meter>, private val desiredPosition: SIUnit<Meter>) : FalconCommand(Climber) {

    override fun initialize() {
        Climber.extend(true)
    }

    override fun execute() {
        Climber.setHeight(desiredHeight)
        AutoHookCommand(desiredPosition)
    }

    override fun end(interrupted: Boolean) {
        Climber.extend(false)
        Climber.setNeutral()
    }
}
