/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.frc2020.subsystems.Drivetrain
import org.ghrobotics.frc2020.subsystems.Turret
import org.ghrobotics.frc2020.vision.VisionProcessing
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.utils.DoubleSource
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

/**
 * A command that aligns the turret to the best available vision target.
 */
class VisionTurretCommand : FalconCommand(Turret) {
    override fun initialize() = VisionProcessing.turnOnLEDs()
    override fun execute() = Turret.setAngle(Turret.angle + SIUnit(VisionProcessing.angle.radians))
    override fun end(interrupted: Boolean) = VisionProcessing.turnOffLEDs()
}

/**
 * A command that can be used for manual control of the turret.
 */
class ManualTurretCommand(val percent: DoubleSource) : FalconCommand(Turret) {
    override fun execute() = Turret.setPercent(percent())
    override fun end(interrupted: Boolean) = Turret.setPercent(0.0)
}

/**
 * A command that zeros the turret when the robot is being setup.
 */
class ZeroTurretCommand : FalconCommand(Turret) {

    private val timer = Timer()

    override fun initialize() = timer.start()

    override fun execute() {
        if (!Turret.hallEffectEngaged) timer.reset()
    }

    override fun end(interrupted: Boolean) {
        timer.stop()
        Turret.zero()
    }

    override fun isFinished() = timer.get() > 3.0
    override fun runsWhenDisabled() = true
}
