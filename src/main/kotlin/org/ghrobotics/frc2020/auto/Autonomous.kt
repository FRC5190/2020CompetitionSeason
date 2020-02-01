/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto

import org.ghrobotics.frc2020.auto.routines.CharacterizationRoutine
import org.ghrobotics.frc2020.auto.routines.CheckSubsystemsRoutine
import org.ghrobotics.frc2020.auto.routines.DefaultRoutine
import org.ghrobotics.frc2020.auto.routines.LineCrossRoutine
import org.ghrobotics.frc2020.auto.routines.TestingRoutine
import org.ghrobotics.frc2020.auto.routines.ThreeBallRoutine
import org.ghrobotics.frc2020.comms.Network

/**
 * Handles the autonomous portion of the game -- the first 15 seconds.
 */
object Autonomous {
    fun start() {
        // Start the auto mode based on what is selected.
        when (Network.autoModeSelector.selected) {
            Mode.CHECK_SUBSYSTEMS -> CheckSubsystemsRoutine()
            Mode.CHARACTERIZE -> CharacterizationRoutine()
            Mode.TESTING -> TestingRoutine()

            Mode.LINE_CROSS -> LineCrossRoutine()

            Mode.THREE_BALL -> ThreeBallRoutine()

            Mode.SIX_BALL_RENDEZVOUS -> TODO()
            Mode.SIX_BALL_TRENCH -> TODO()

            Mode.EIGHT_BALL_RENDEZVOUS -> TODO()
            Mode.EIGHT_BALL_TRENCH -> TODO()
            Mode.EIGHT_BALL_STEAL -> TODO()

            Mode.TEN_BALL_STEAL -> TODO()

            null -> DefaultRoutine()
        }.startRoutine()
    }

    /**
     * Represents the various auto modes.
     */
    enum class Mode {
        // Subsystem Checking
        CHECK_SUBSYSTEMS,
        // Drivetrain Characterization
        CHARACTERIZE,
        // Testing
        TESTING,
        // Cross the Line
        LINE_CROSS,
        // Basic 3 Ball
        THREE_BALL,
        // 6 Ball Autos
        SIX_BALL_RENDEZVOUS,
        SIX_BALL_TRENCH,
        // 8 Ball Autos
        EIGHT_BALL_RENDEZVOUS,
        EIGHT_BALL_TRENCH,
        EIGHT_BALL_STEAL,
        // 10 Ball Autos
        TEN_BALL_STEAL

    }
}
