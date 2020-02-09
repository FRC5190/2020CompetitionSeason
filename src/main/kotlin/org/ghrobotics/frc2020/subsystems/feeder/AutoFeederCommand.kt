/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.feeder

import org.ghrobotics.lib.commands.FalconCommand

class AutoFeederCommand(val buttonState: Boolean) : FalconCommand(Feeder) {
    var lastIntakeValue = false
    var lastTurretValue = false
    var lastButtonState = false

    override fun execute() {
        // If there is a ball on intake side of feeder and feeder is not full, intake
        if (Feeder.intakeValue && Feeder.ballCount < 5 && Feeder.status == Feeder.Status.INTAKE) {
            Feeder.setPercent(0.6)
        }

        // If a ball has been taken in, increase count of balls
        if (!Feeder.intakeValue && Feeder.intakeValue != lastIntakeValue && Feeder.status == Feeder.Status.INTAKE) {
            Feeder.ballCount++
        }

        // If a ball has been shot with the turret, decrease count of balls
        if (!Feeder.turretValue && Feeder.turretValue != lastTurretValue) {
            Feeder.ballCount--
            // If the turret has shot all balls in feeder, set feeder to intake mode
            if (Feeder.ballCount == 0) {
                Feeder.status = Feeder.Status.INTAKE
            }
        }

        // When driver pushes button, switch feeder mode
        if (buttonState && buttonState != lastButtonState) {
            Feeder.status = if (Feeder.status == Feeder.Status.INTAKE) {
                Feeder.Status.TURRET
            } else {
                if (Feeder.ballCount == 0) {
                    Feeder.Status.INTAKE
                } else {
                    Feeder.Status.TOINTAKE
                }
            }
        }

        // When set to cycle back to intake mode, drive feeder backwards until balls reach intake side
        if (Feeder.ballCount != 0 && Feeder.status == Feeder.Status.TOINTAKE && !Feeder.intakeValue) {
            if (Feeder.intakeValue) {
                Feeder.setPercent(0.0)
                Feeder.ballCount--
                Feeder.status = Feeder.Status.INTAKE
            } else {
                Feeder.setPercent(-1.0)
            }
        }

        // Get the balls in the feeder ready for the turret when the feeder is in turret mode
        if (Feeder.ballCount != 0 && Feeder.status == Feeder.Status.TURRET && !Feeder.turretValue) {
            Feeder.setPercent(1.0)
        }

        lastIntakeValue = Feeder.intakeValue
        lastTurretValue = Feeder.turretValue
        lastButtonState = buttonState
    }
}
