/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.turret

import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.utils.Source

/**
 * A command that sets the angle of the turret to a specific value.
 */
class AutoTurretCommand(private val angle: Source<SIUnit<Radian>>) : FalconCommand(Turret) {
    override fun execute() = Turret.setAngle(angle())

    @Suppress("MemberVisibilityCanBePrivate")
    companion object {
        /**
         * Creates a TurretCommand from a field-oriented goal.
         *
         * @param fieldRelativeAngle The field-relative angle.
         */
        fun createFromFieldOrientedAngle(fieldRelativeAngle: SIUnit<Radian>): AutoTurretCommand {
            return AutoTurretCommand { fieldRelativeAngle - SIUnit(Drivetrain.getPose().rotation.radians) }
        }

        /**
         * Creates a TurretCommand from a field-oriented goal.
         *
         * @param fieldRelativeAngle The field-relative angle.
         */
        fun createFromFieldOrientedAngle(fieldRelativeAngle: Rotation2d): AutoTurretCommand =
            createFromFieldOrientedAngle(SIUnit(fieldRelativeAngle.radians))
    }
}
