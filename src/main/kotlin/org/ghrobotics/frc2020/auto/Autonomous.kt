/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto

import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.auto.routines.CharacterizationRoutine
import org.ghrobotics.frc2020.auto.routines.CheckSubsystemsRoutine
import org.ghrobotics.frc2020.auto.routines.DefaultRoutine
import org.ghrobotics.frc2020.auto.routines.LineCrossRoutine
import org.ghrobotics.frc2020.auto.routines.RendezvousTrenchRoutine
import org.ghrobotics.frc2020.auto.routines.StealRoutine
import org.ghrobotics.frc2020.auto.routines.TestingRoutine
import org.ghrobotics.frc2020.auto.routines.TrenchRoutine
import org.ghrobotics.frc2020.comms.Network

/**
 * Handles the autonomous portion of the game -- the first 15 seconds.
 */
object Autonomous {

    // Empty string to make the meme work.
    private const val IT = ""

    fun start() {
        // Start the auto mode based on what is selected.
        val JUST = when (Network.autoModeSelector.selected) {
            Mode.CHECK_SUBSYSTEMS -> CheckSubsystemsRoutine()
            Mode.CHARACTERIZE -> CharacterizationRoutine()
            Mode.TESTING -> TestingRoutine()

            Mode.LINE_CROSS -> LineCrossRoutine()

            Mode.THREE_BALL -> TODO()

            Mode.SIX_BALL_RENDEZVOUS -> TODO()
            Mode.SIX_BALL_TRENCH -> TrenchRoutine(TrenchRoutine.Type.SIX_BALL)

            Mode.EIGHT_BALL_RENDEZVOUS -> TODO()
            Mode.EIGHT_BALL_TRENCH -> TrenchRoutine(TrenchRoutine.Type.EIGHT_BALL)
            Mode.EIGHT_BALL_STEAL -> StealRoutine(StealRoutine.Type.EIGHT_BALL)

            Mode.TEN_BALL_STEAL -> StealRoutine(StealRoutine.Type.TEN_BALL)
            Mode.TEN_BALL_NEAR -> RendezvousTrenchRoutine()

            null -> DefaultRoutine()
        }.getRoutine()

        // Start auto.
        JUST S3ND IT
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
        TEN_BALL_STEAL,
        TEN_BALL_NEAR
    }

    @Suppress("FunctionName")
    private infix fun Command.S3ND(any: Any) = schedule()
}
