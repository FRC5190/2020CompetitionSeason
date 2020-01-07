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

class TestFortuneWheelCommand : FalconCommand(FortuneWheelSpinner) {
    var status = Status.STANDBY

    init {

    }

    override fun initialize() {
        // todo
    }

    override fun execute() {
        // todo
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
