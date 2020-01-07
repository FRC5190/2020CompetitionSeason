/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.Robot
import org.ghrobotics.frc2020.auto.AutoRoutine

/**
 * Tests all subsystems by running their respective checks.
 */
class TestSubsystemsRoutine : AutoRoutine {
    /**
     * Return the test subsystem routine. There is nothing in here because
     * we are manually starting the test routine in the startRoutine() method.
     */
    override fun getRoutine(): Command {
        return Robot.getChecks()
    }
}
