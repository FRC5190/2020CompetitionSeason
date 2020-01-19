/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.fortunewheel

import org.ghrobotics.frc2020.subsystems.SubsystemTestManager
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.meters

class TestFortuneWheelCommand : FalconCommand(FortuneWheel) {
    var status = Status.STANDBY

    override fun initialize() {
        FortuneWheel.resetPosition()
        status = Status.MOTOR
    }

    override fun execute() {
        if (FortuneWheel.spinnerPosition < 1.meters) {
            FortuneWheel.setPercent(0.2)
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
