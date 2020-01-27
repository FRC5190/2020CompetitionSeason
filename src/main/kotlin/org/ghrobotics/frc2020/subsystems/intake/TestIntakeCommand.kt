/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.intake

import org.ghrobotics.frc2020.subsystems.SubsystemTestManager
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

class TestIntakeCommand : FalconCommand(Intake) {
    private var success = true

    override fun initialize() {
        Intake.resetPosition(0.nativeUnits)
        Intake.extendPiston(true)
    }

    override fun execute() {
        Intake.setPercent(.75)
    }

    override fun end(interrupted: Boolean) {
        Intake.extendPiston(false)
        success = Intake.position > 2.nativeUnits
        Intake.setNeutral()
        SubsystemTestManager.intakeCheck = success
    }
}
