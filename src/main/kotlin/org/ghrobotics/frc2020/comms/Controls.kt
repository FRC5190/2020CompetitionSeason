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
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.ghrobotics.frc2020.Robot
import org.ghrobotics.frc2020.subsystems.turret.TurretConstants
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.climber.Climber
import org.ghrobotics.frc2020.subsystems.climber.ClimberPercentCommand
import org.ghrobotics.frc2020.subsystems.fortunewheel.FortuneColor
import org.ghrobotics.frc2020.subsystems.fortunewheel.FortuneWheel
import org.ghrobotics.frc2020.subsystems.fortunewheel.FortuneWheelPositionCommand
import org.ghrobotics.frc2020.subsystems.hook.HookPercentCommand
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.frc2020.subsystems.turret.TurretPositionCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.utils.map
import org.ghrobotics.lib.utils.not
import org.ghrobotics.lib.wrappers.hid.button
import org.ghrobotics.lib.wrappers.hid.kA
import org.ghrobotics.lib.wrappers.hid.kB
import org.ghrobotics.lib.wrappers.hid.kBack
import org.ghrobotics.lib.wrappers.hid.kBumperLeft
import org.ghrobotics.lib.wrappers.hid.kBumperRight
import org.ghrobotics.lib.wrappers.hid.kY
import org.ghrobotics.lib.wrappers.hid.triggerAxisButton
import org.ghrobotics.lib.wrappers.hid.xboxController

/**
 * Contains all the teleop controls for the robot.
 */
object Controls {
    val driverController = xboxController(0) {

        /**
         * Represents controls when the robot is in climb mode.
         */
        state(Robot::isClimbMode) {
            /**
             * Retract the climber when the left bumper is pressed.
             */
            button(kBumperLeft).changeOn { Climber.extend(false) }

            /**
             * Extend the climber the right bumper is pressed.
             */
            button(kBumperRight).changeOn { Climber.extend(true) }

            /**
             * Pull the robot up when the left trigger is pressed.
             */
            triggerAxisButton(GenericHID.Hand.kLeft, threshold = 0.04) {
                change(ClimberPercentCommand(source))
            }

            /**
             * Pull the robot down when the right trigger is pressed.
             */
            triggerAxisButton(GenericHID.Hand.kRight, threshold = 0.04) {
                change(ClimberPercentCommand(source.map { it * -1.0 }))
            }

            axisButton(XboxController.Axis.kRightY.value) {
                change(HookPercentCommand(source))
            }

            /**
             * Toggles the winch brake.
             */
            button(kA).changeOn { Climber.setWinchBrake(!Climber.isWinchLocked) }
        }

        /**
         * Represents controls when the robot is not in climb mode.
         */
        state(!Robot::isClimbMode) {
            /**
             * Aim the turret and shoot power cells when the right bumper is pressed.
             * The turret and shooter will aim and the feeder will feed all balls
             * to the shooter when the drivetrain comes to a complete stop.
             */
            button(kBumperRight).change(Superstructure.scoreWhenStopped())

            button(kBumperLeft).change(Superstructure.intake())
            triggerAxisButton(GenericHID.Hand.kLeft).change(Superstructure.release())

            /**
             * Jogs the turret zero a certain amount. This is useful when vision
             * align is consistently skewed to one side. Hitting the left
             * POV is useful when the turret is constantly aligning to the
             * right and vice versa.
             */
            pov(270).changeOn { Turret.jogZero(TurretConstants.kDefaultJogAmount) }
            pov(90).changeOn { Turret.jogZero(-TurretConstants.kDefaultJogAmount) }

            /**
             * Perform rotation control when the POV up button is pressed.
             */
            pov(0).change(FortuneWheelPositionCommand(28))

            /**
             * Perform position control when the POV down button is pressed.
             */
            pov(180).change(FortuneWheelPositionCommand { GameData.getColor() ?: FortuneColor.RED })
        }

        /**
         * Toggles climb mode. When climb mode is active, buttons which
         * allow operation of the climb subsystem are activated.
         */
        button(kB).changeOn {
            Robot.isClimbMode = !Robot.isClimbMode
            if (Robot.isClimbMode) {
                TurretPositionCommand { 55.degrees }.schedule()
            } else {
                Turret.defaultCommand.schedule()
            }
        }

        button(kY).changeOn {
            Robot.isFortuneWheelMode = !Robot.isFortuneWheelMode
            if (Robot.isFortuneWheelMode) {
                FortuneWheel.extendSpinnerPiston(true)
            } else {
                FortuneWheel.extendSpinnerPiston(false)
            }
        }

        button(kBack).changeOn {
            CommandScheduler.getInstance().cancelAll()
        }
    }

    fun update() {
        driverController.update()
    }
}
