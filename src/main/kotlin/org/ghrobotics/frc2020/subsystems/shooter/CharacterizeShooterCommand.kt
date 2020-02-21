/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.shooter

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.inRadians

class CharacterizeShooterCommand : FalconCommand(Shooter) {

    private val numberArray = DoubleArray(6)

    private val autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed")
    private val telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry")

    private var priorAutoSpeed = 0.0

    override fun execute() {
        // Get the current timestamp.
        val now = Timer.getFPGATimestamp()

        // Get position and velocity.
        val position = Shooter.position
        val velocity = Shooter.velocity

        // Get battery voltage.
        val battery = RobotController.getBatteryVoltage()

        // Get the voltage.
        val motorVolts = Shooter.voltage

        // Retrieve commanded speed from NT
        val autoSpeed = autoSpeedEntry.getDouble(0.0)
        priorAutoSpeed = autoSpeed

        // Command motors
        Shooter.setPercent(autoSpeed)

        // Send telemetry
        numberArray[0] = now
        numberArray[1] = battery
        numberArray[2] = autoSpeed
        numberArray[3] = motorVolts.value
        numberArray[4] = position.inRadians()
        numberArray[5] = velocity.value

        telemetryEntry.setNumberArray(numberArray.toTypedArray())
    }
}
