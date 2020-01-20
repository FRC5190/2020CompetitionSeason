/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import org.ghrobotics.frc2020.subsystems.Climber
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit

class ClosedLoopClimberCommand(private val desiredHeight: SIUnit<NativeUnit>) : FalconCommand(Climber) {

    override fun initialize() {
        Climber.pistonBrake.set(false)
        Climber.setHeight(desiredHeight)
    }

    override fun end(interrupted: Boolean) {
        Climber.pistonBrake.set(true)
        Climber.ClimberMasterMotor.setNeutral()
        Climber.ClimberSlaveMotor.setNeutral()
    }
}
