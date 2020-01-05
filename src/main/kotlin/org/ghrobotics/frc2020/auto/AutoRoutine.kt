/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto

import edu.wpi.first.wpilibj2.command.Command

/**
 * Interface for all autonomous routines.
 */
interface AutoRoutine {

    /**
     * Returns the specific auto routine in the form of a command.
     */
    fun getRoutine(): Command

    /**
     * Starts the auto routine.
     */
    fun startRoutine() {
        getRoutine().schedule()
    }
}
