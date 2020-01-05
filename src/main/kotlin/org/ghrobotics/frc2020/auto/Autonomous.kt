/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto

import org.ghrobotics.frc2020.Network
import org.ghrobotics.frc2020.auto.routines.CharacterizationRoutine
import org.ghrobotics.frc2020.auto.routines.DefaultRoutine
import org.ghrobotics.frc2020.auto.routines.TestSubsystemsRoutine

/**
 * Handles the autonomous portion of the game -- the first 15 seconds.
 */
object Autonomous {
    fun start() {
        // Start the auto mode based on what is selected.
        when (Network.autoModeSelector.selected) {
            Mode.TEST -> TestSubsystemsRoutine()
            Mode.CHARACTERIZE -> CharacterizationRoutine()
            null -> DefaultRoutine()
        }.startRoutine()
    }

    /**
     * Represents the various auto modes.
     */
    enum class Mode {
        TEST, CHARACTERIZE
    }
}
