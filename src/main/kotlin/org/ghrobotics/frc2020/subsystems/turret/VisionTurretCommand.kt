/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.turret

import org.ghrobotics.frc2020.TurretConstants
import org.ghrobotics.frc2020.vision.VisionProcessing
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.inRadians

/**
 * A command that aligns the turret to the best available vision target.
 */
class VisionTurretCommand : FalconCommand(Turret) {
    override fun initialize() = VisionProcessing.turnOnLEDs()
    override fun execute() {
        Turret.setAngle(
            Turret.angle + SIUnit(
                VisionProcessing.angle.radians +
                    TurretConstants.kBadTurretOffset.inRadians()
            )
        )
    }

    override fun end(interrupted: Boolean) = VisionProcessing.turnOffLEDs()
}
