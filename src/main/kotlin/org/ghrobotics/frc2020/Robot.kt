/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.ghrobotics.frc2020.auto.Autonomous
import org.ghrobotics.frc2020.subsystems.Drivetrain
import org.ghrobotics.frc2020.subsystems.FortuneWheelSpinner
import org.ghrobotics.lib.wrappers.FalconTimedRobot

/**
 * Main robot class.
 */
object Robot : FalconTimedRobot() {

    // Constructor of the Robot class.
    init {
        // Add the subsystems to the subsystem handler
        +Drivetrain
        +FortuneWheelSpinner
    }

    // Runs once when robot boots up
    override fun robotInit() {
    }

    // Runs once when autonomous period starts
    override fun autonomousInit() {
        Autonomous.start()
    }

    // Runs once when teleop period starts
    override fun teleopInit() {}

    // Runs once when robot is disabled
    override fun disabledInit() {}

    // Runs every 20 ms when robot is on
    override fun robotPeriodic() {
        Shuffleboard.update()
    }

    // Runs every 20 ms when autonomous is enabled
    override fun autonomousPeriodic() {}

    // Runs every 20 ms when teleop is enabled
    override fun teleopPeriodic() {}

    // Runs every 20 ms when robot is disabled
    override fun disabledPeriodic() {}

    fun getChecks() = getSubsystemChecks()
}
