/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.feeder

import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.commands.FalconCommand

/**
 * Automatically moves the feeder based on sensor data.
 */
class FeederPositionCommand : FalconCommand(Feeder) {

    // Keeps track of the time when the sensor was last triggered.
    private var sensorLastTriggered = -1.0

    override fun execute() {
        if (Feeder.intakeSensorTriggered) {
            sensorLastTriggered = Timer.getFPGATimestamp()
        }

        if (Timer.getFPGATimestamp() - sensorLastTriggered < 0.05) {
            if (Feeder.exitSensorTriggered) {
                Feeder.setPercent(0.2, 1.0)
            } else {
                Feeder.setPercent(0.6, 1.0)
            }
        } else {
            Feeder.setPercent(0.0, 1.0)
        }

        if (Feeder.exitSensorTriggered) {
            Feeder.setExitPiston(true)
        }
    }

    override fun end(interrupted: Boolean) {
        Feeder.setNeutral()
    }
}
