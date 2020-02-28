/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto

import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.auto.routines.CheckSubsystemsRoutine
import org.ghrobotics.frc2020.auto.routines.DefaultRoutine
import org.ghrobotics.frc2020.auto.routines.DriveCharacterizationRoutine
import org.ghrobotics.frc2020.auto.routines.EightBallReverseTrenchRoutine
import org.ghrobotics.frc2020.auto.routines.EightBallTrenchRoutine
import org.ghrobotics.frc2020.auto.routines.FiveBallStealRoutine
import org.ghrobotics.frc2020.auto.routines.LineCrossRoutine
import org.ghrobotics.frc2020.auto.routines.ShooterCharacterizationRoutine
import org.ghrobotics.frc2020.auto.routines.SixBallReverseTrenchRoutine
import org.ghrobotics.frc2020.auto.routines.SixBallTrenchRoutine
import org.ghrobotics.frc2020.auto.routines.TenBallStealRoutine
import org.ghrobotics.frc2020.auto.routines.TestingRoutine
import org.ghrobotics.frc2020.auto.routines.ThreeBallRoutine
import org.ghrobotics.frc2020.comms.Network

/**
 * Handles the autonomous portion of the game -- the first 15 seconds.
 */
object Autonomous {

    // Empty string to make the meme work.
    private const val IT = ""

    fun start() {
        // Start the auto mode based on what is selected.
        @Suppress("LocalVariableName") val JUST = when (Network.autoModeSelector.selected) {
            Mode.CHECK_SUBSYSTEMS -> CheckSubsystemsRoutine()
            Mode.DRIVE_CHARACTERIZE -> DriveCharacterizationRoutine()
            Mode.SHOOTER_CHARACTERIZE -> ShooterCharacterizationRoutine()

            Mode.TESTING -> TestingRoutine()
            Mode.LINE_CROSS -> LineCrossRoutine()
            Mode.THREE_BALL -> ThreeBallRoutine()

            Mode.FIVE_BALL_STEAL -> FiveBallStealRoutine()

            Mode.SIX_BALL_TRENCH -> SixBallTrenchRoutine()
            Mode.REVERSE_SIX_BALL_TRENCH -> SixBallReverseTrenchRoutine()
            Mode.ASSIST_WITH_REVERSE_SIX_BALL_TRENCH -> SixBallReverseTrenchRoutine(pushOff = true)


            Mode.EIGHT_BALL_TRENCH -> EightBallTrenchRoutine()
            Mode.REVERSE_EIGHT_BALL_TRENCH -> EightBallReverseTrenchRoutine()

            Mode.TEN_BALL_STEAL -> TenBallStealRoutine()

            null -> DefaultRoutine()
        }.getRoutine()

        // Start auto.
        JUST S3ND IT
    }

    /**
     * Represents the various auto modes.
     */
    enum class Mode {
        CHECK_SUBSYSTEMS,
        DRIVE_CHARACTERIZE,
        SHOOTER_CHARACTERIZE,

        TESTING,
        LINE_CROSS,
        THREE_BALL,

        FIVE_BALL_STEAL,

        SIX_BALL_TRENCH,
        REVERSE_SIX_BALL_TRENCH,
        ASSIST_WITH_REVERSE_SIX_BALL_TRENCH,

        EIGHT_BALL_TRENCH,
        REVERSE_EIGHT_BALL_TRENCH,

        TEN_BALL_STEAL
    }

    @Suppress("FunctionName")
    private infix fun Command.S3ND(any: Any) = schedule()
}
