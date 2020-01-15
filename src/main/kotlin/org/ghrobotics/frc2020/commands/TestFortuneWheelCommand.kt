/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import org.ghrobotics.frc2020.subsystems.FortuneWheelSpinner
import org.ghrobotics.frc2020.subsystems.SubsystemTestManager
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.meters

class TestFortuneWheelCommand : FalconCommand(FortuneWheelSpinner) {
    var status = Status.STANDBY

    override fun initialize() {
        FortuneWheelSpinner.resetPosition()
        status = Status.MOTOR
    }

    override fun execute() {
        if (FortuneWheelSpinner.spinnerPosition < 1.meters) {
            FortuneWheelSpinner.setVelocity(1.meters.velocity)
        } else {
            status = Status.SUCCESS
        }
    }

    override fun end(interrupted: Boolean) {
        var success = status == Status.SUCCESS
        SubsystemTestManager.fortuneCheck = success
    }

    enum class Status {
        STANDBY,
        MOTOR,
        SUCCESS
    }
}
