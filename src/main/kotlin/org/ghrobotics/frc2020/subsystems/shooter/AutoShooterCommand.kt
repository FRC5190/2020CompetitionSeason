/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.shooter

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.AngularVelocity
import org.ghrobotics.lib.utils.Source

/**
 * A command that sets the speed of the shooter.
 */
class AutoShooterCommand(val speed: Source<SIUnit<AngularVelocity>>) : FalconCommand(Shooter) {
    override fun execute() = Shooter.setSpeed(speed())
    override fun end(interrupted: Boolean) = Shooter.setNeutral()
}
