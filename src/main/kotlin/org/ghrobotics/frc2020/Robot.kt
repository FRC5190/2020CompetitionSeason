/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020

import org.ghrobotics.frc2020.auto.Autonomous
import org.ghrobotics.frc2020.auto.TrajectoryManager
import org.ghrobotics.frc2020.comms.Controls
import org.ghrobotics.frc2020.comms.Network
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.climber.Climber
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.feeder.Feeder
import org.ghrobotics.frc2020.subsystems.fortunewheel.FortuneWheel
import org.ghrobotics.frc2020.subsystems.hood.Hood
import org.ghrobotics.frc2020.subsystems.hook.Hook
import org.ghrobotics.frc2020.subsystems.intake.Intake
import org.ghrobotics.frc2020.subsystems.led.LED
import org.ghrobotics.frc2020.subsystems.shooter.Shooter
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.frc2020.subsystems.turret.TurretZeroCommand
import org.ghrobotics.frc2020.vision.LimelightManager
import org.ghrobotics.lib.wrappers.FalconTimedRobot

/**
 * Main robot class.
 */
object Robot : FalconTimedRobot() {

    // Whether the robot is in climb mode.
    var isClimbMode = false

    // Whether the robot is in fortune wheel mode.
    var isFortuneWheelMode = false

    // Runs once when robot boots up
    override fun robotInit() {
        // Initialize auto paths
        TrajectoryManager

        // Initialize Network
        Network

        // Add vision processing
        +LimelightManager

        // Add subsystems
        +Climber
        +Drivetrain
        +Feeder
        +FortuneWheel
        +Hood
        +Hook
        +Intake
        +LED
        +Shooter
        +Turret

        TurretZeroCommand().schedule()
    }

    // Runs once when autonomous period starts
    override fun autonomousInit() {
        Drivetrain.setBrakeMode(true)
        Turret.setBrakeMode(true)
        Autonomous.start()
    }

    // Runs once when teleop period starts
    override fun teleopInit() {
        Autonomous.cancel()
        Drivetrain.setBrakeMode(true)
        Turret.setBrakeMode(true)
    }

    // Runs once when robot is disabled
    override fun disabledInit() {
        Drivetrain.setBrakeMode(false)
        Turret.setBrakeMode(false)
    }

    // Runs every 20 ms when robot is on
    override fun robotPeriodic() {}

    // Runs every 20 ms when autonomous is enabled
    override fun autonomousPeriodic() {
    }

    // Runs every 20 ms when teleop is enabled
    override fun teleopPeriodic() {
        Controls.update()
    }

    // Runs every 20 ms when robot is disabled
    override fun disabledPeriodic() {
        Controls.update()
    }

    fun getChecks() = getSubsystemChecks()
}
