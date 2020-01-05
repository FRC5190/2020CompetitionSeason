/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import edu.wpi.first.wpilibj.util.Color
import org.ghrobotics.frc2020.FortuneWheelConstants
import org.ghrobotics.frc2020.subsystems.FortuneWheelSpinner
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.operations.times

class FortuneWheelRotationCommand : FalconCommand(FortuneWheelSpinner) {
    var status = Status.STOPPED
    var rotations = 0.0

    override fun initialize() {
        status = Status.RUNNING
        FortuneWheelSpinner.resetPosition()
    }

    override fun execute() {
        if (status == Status.RUNNING) {
            // Set motor to spin
            FortuneWheelSpinner.setOutput((FortuneWheelConstants.kFortuneWheelSpeed.value * FortuneWheelConstants.kFortuneWheelRadius).velocity)
            rotations = FortuneWheelSpinner.spinnerPosition.value / (360.degrees.value * FortuneWheelConstants.kFortuneWheelRadius.value)
        }

        if (rotations > 3.0) {
            status = Status.STOPPED
            FortuneWheelSpinner.setOutput(0.meters.velocity)
        }
    }

    enum class Status {
        RUNNING,
        STOPPED
    }
}
