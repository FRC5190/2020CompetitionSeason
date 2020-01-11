/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020

import org.ghrobotics.frc2020.Robot.unaryPlus
import org.ghrobotics.frc2020.auto.Autonomous
import org.ghrobotics.frc2020.auto.Paths
import org.ghrobotics.frc2020.comms.Controls
import org.ghrobotics.frc2020.comms.Network
import org.ghrobotics.frc2020.subsystems.LEDSubsystem
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.shooter.Shooter
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.frc2020.subsystems.turret.ZeroTurretCommand
import org.ghrobotics.frc2020.vision.VisionProcessing
import org.ghrobotics.lib.wrappers.FalconTimedRobot

/**
 * Main robot class.
 */
object Robot : FalconTimedRobot() {

    // Constructor of the Robot class.
    init {
        // Initialize auto paths
        Paths

        // Initialize Network
        Network

        // Add vision processing
        +VisionProcessing

        // Add subsystems
        +Drivetrain
        +Shooter
        +Turret
        +LEDSubsystem
    }

    // Runs once when robot boots up
    override fun robotInit() {
        LEDSubsystem.setStatus(LEDStatusConstants.ROBOT_INITIALIZING)
        ZeroTurretCommand().schedule()
    }

    // Runs once when autonomous period starts
    override fun autonomousInit() {
        Drivetrain.setBrakeMode(true)
        Autonomous.start()
    }

    // Runs once when teleop period starts
    override fun teleopInit() {
        Drivetrain.setBrakeMode(true)
    }

    // Runs once when robot is disabled
    override fun disabledInit() {
        Drivetrain.setBrakeMode(false)
    }

    // Runs every 20 ms when robot is on
    override fun robotPeriodic() {
        Controls.update()
    }

    // Runs every 20 ms when autonomous is enabled
    override fun autonomousPeriodic() {}

    // Runs every 20 ms when teleop is enabled
    override fun teleopPeriodic() {}

    // Runs every 20 ms when robot is disabled
    override fun disabledPeriodic() {}

    fun getChecks() = getSubsystemChecks()
}
