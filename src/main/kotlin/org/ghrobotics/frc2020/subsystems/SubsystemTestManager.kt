/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems

/**
 * Manages running and storing the state of subsystem tests.
 */
object SubsystemTestManager {
    // States for each subsystem test.
    var drivetrainCheck = true
    var intakeCheck = true
    var shooterCheck = true

    /**
     * Returns whether all checks have passed.
     * @return Whether all checks have passed.
     */
    fun haveAllChecksPassed(): Boolean {
        return drivetrainCheck && shooterCheck && intakeCheck
    }

    /**
     * Resets all subsystem test states to true.
     */
    fun reset() {
        drivetrainCheck = true
        intakeCheck = true
        shooterCheck = true
    }
}
