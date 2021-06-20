/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto

import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.auto.routines.DefaultRoutine
import org.ghrobotics.frc2020.auto.routines.DrivetrainCharacterizationRoutine
import org.ghrobotics.frc2020.auto.routines.ShooterCharacterizationRoutine
import org.ghrobotics.frc2020.auto.routines.TrenchRendezvousRoutine
import org.ghrobotics.frc2020.comms.Network
import org.ghrobotics.lib.commands.sequential

/**
 * Handles the autonomous portion of the game -- the first 15 seconds.
 */
object Autonomous {

    // Empty string to make the meme work.
    private const val IT = ""
    private var JUST: Command = sequential { }

    fun start() {
        // Start the auto mode based on what is selected.
        JUST = when (Network.autoModeSelector.selected) {
            // Characterization Routines.
            Mode.DRIVETRAIN_CHARACTERIZATION -> DrivetrainCharacterizationRoutine()
            Mode.SHOOTER_CHARACTERIZATION -> ShooterCharacterizationRoutine()

            // Trench Routines.
            Mode.TRENCH_AND_RENDEZVOUS -> TrenchRendezvousRoutine()

            null -> DefaultRoutine()
        }.getRoutine()

        // Start auto.
        JUST S3ND IT
    }

    fun cancel() {
        JUST.cancel()
    }

    /**
     * Represents the various auto modes.
     */
    enum class Mode {
        DRIVETRAIN_CHARACTERIZATION, SHOOTER_CHARACTERIZATION,
        TRENCH_AND_RENDEZVOUS
    }

    @Suppress("FunctionName")
    private infix fun Command.S3ND(any: Any) = schedule()
}
