/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import org.ghrobotics.frc2020.subsystems.Shooter
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.AngularVelocity
import org.ghrobotics.lib.utils.DoubleSource
import org.ghrobotics.lib.utils.Source

/**
 * A command that sets the speed of the shooter.
 */
class AutoShooterCommand(val speed: Source<SIUnit<AngularVelocity>>) : FalconCommand(Shooter) {
    override fun execute() = Shooter.setSpeed(speed())
}

/**
 * A command that allows manual control of the shooter.
 */
class ManualShooterCommand(val source: DoubleSource) : FalconCommand(Shooter) {
    override fun execute() {
        println(source() * 12)
        Shooter.setPercent(source())
    }
}
