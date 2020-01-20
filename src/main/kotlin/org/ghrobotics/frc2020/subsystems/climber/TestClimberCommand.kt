/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems.climber

import org.ghrobotics.frc2020.subsystems.SubsystemTestManager
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

class TestClimberCommand : FalconCommand(Climber) {
    private var success = true

    override fun initialize() {
        Climber.setBrake(false)
        Climber.climberMasterMotor.encoder.resetPosition(0.meters)
        Climber.climberSlaveMotor.encoder.resetPosition(0.meters)
    }
    override fun execute() {
        OpenLoopClimberCommand { .75 }
    }

    override fun end(interrupted: Boolean) {
        Climber.setBrake(true)
        success = Climber.position > 2.meters
        Climber.setNeutral()
        SubsystemTestManager.climberCheck = success
    }
}
