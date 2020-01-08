/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.frc2020.subsystems.Drivetrain
import org.ghrobotics.frc2020.subsystems.Turret
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit

class TurretCommand(private val angle: Rotation2d) : FalconCommand(Turret) {

    override fun initialize() {
        Turret.setAngle(SIUnit(angle.radians))
    }

    companion object {
        fun createFromFieldOrientedAngle(angle: Rotation2d): TurretCommand {
            val robotAngle = Drivetrain.getPose().rotation
            return TurretCommand(angle - robotAngle)
        }
    }
}
