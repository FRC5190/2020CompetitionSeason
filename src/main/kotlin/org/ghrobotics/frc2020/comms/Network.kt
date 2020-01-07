/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.comms

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.ghrobotics.frc2020.auto.Autonomous
import org.ghrobotics.lib.wrappers.networktables.enumSendableChooser

/**
 * Handles all networking to Shuffleboard, including diagnostics from
 * various subsystems.
 */
object Network {
    // Shuffleboard Tab
    private val tab = Shuffleboard.getTab("5190")

    // Sendable Chooser for auto mode.
    val autoModeSelector = enumSendableChooser<Autonomous.Mode>()

    init {
        // Add the selector to Shuffleboard.
        tab.add("Auto Mode Selector",
            autoModeSelector
        )
    }
}
