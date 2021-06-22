/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.comms

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import org.ghrobotics.frc2020.Robot
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.climber.Climber
import org.ghrobotics.frc2020.subsystems.climber.ClimberPercentCommand
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.hook.HookPercentCommand
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.frc2020.subsystems.turret.TurretPositionCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.utils.map
import org.ghrobotics.lib.utils.not
import org.ghrobotics.lib.wrappers.hid.button
import org.ghrobotics.lib.wrappers.hid.kA
import org.ghrobotics.lib.wrappers.hid.kB
import org.ghrobotics.lib.wrappers.hid.kBumperLeft
import org.ghrobotics.lib.wrappers.hid.kBumperRight
import org.ghrobotics.lib.wrappers.hid.triggerAxisButton
import org.ghrobotics.lib.wrappers.hid.xboxController

/**
 * Contains all the teleop controls for the robot.
 */
object Controls {

    val driverController = xboxController(0) {
        // Controls in regular mode.
        state(!Robot::isClimbMode) {
            // Right bumper to shoot.
            button(kBumperRight).change(Superstructure.scoreWhenStopped())

            // Left bumper to intake power cells.
            button(kBumperLeft).change(Superstructure.intake())

            // Left trigger to exhaust balls through the feeder.
            triggerAxisButton(GenericHID.Hand.kLeft, threshold = 0.04).change(Superstructure.release())

            // POV 0 to force turret field-relative zero.
            pov(0).change(TurretPositionCommand { -Drivetrain.getAngle() })

            // POV 180 to force turret field-relative 180.
            pov(180).change(TurretPositionCommand { -Drivetrain.getAngle() + 180.degrees })
        }

        // Controls in climb mode.
        state(Robot::isClimbMode) {
            // Retract the climber when the left bumper is pressed.
            button(kBumperLeft).changeOn { Climber.extend(false) }

            // Extend the climber the right bumper is pressed.
            button(kBumperRight).changeOn { Climber.extend(true) }

            // Pull the robot up when the left trigger is pressed.
            triggerAxisButton(GenericHID.Hand.kLeft, threshold = 0.04) {
                change(ClimberPercentCommand(source))
            }

            // Pull the robot down when the right trigger is pressed.
            triggerAxisButton(GenericHID.Hand.kRight, threshold = 0.04) {
                change(ClimberPercentCommand(source.map { it * -1.0 }))
            }

            // Move the hook with the right axis.
            axisButton(XboxController.Axis.kRightY.value, threshold = 0.04) {
                change(HookPercentCommand(source))
            }

            // Toggle winch brake.
            button(kA).changeOn { Climber.setWinchBrake(!Climber.isWinchLocked) }
        }

        // Toggle climb mode.
        button(kB).changeOn {
            Robot.isClimbMode = !Robot.isClimbMode
            if (Robot.isClimbMode) {
                TurretPositionCommand { 55.degrees }.schedule()
            } else {
                Turret.defaultCommand.schedule()
            }
        }
    }

    fun update() {
        driverController.update()
    }
}
