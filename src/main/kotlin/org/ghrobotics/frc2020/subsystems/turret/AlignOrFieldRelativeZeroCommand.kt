/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.turret

import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.vision.VisionProcessing
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees

class AlignOrFieldRelativeZeroCommand : FalconCommand(Turret) {

    override fun execute() {
        if (VisionProcessing.isValid) {
            Turret.setAngle(Turret.getAngle() + SIUnit(VisionProcessing.angle.radians) + 3.degrees)
        } else {
            Turret.setAngle(-SIUnit<Radian>(Drivetrain.getPose().rotation.radians))
        }
    }
}