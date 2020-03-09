/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.auto.TrajectoryManager
import org.ghrobotics.frc2020.auto.WaypointManager
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.sequential

class StealRoutine(private val shootFromProtected: Boolean = false) : AutoRoutine {

    // Paths
    private val path1 = TrajectoryManager.stealStartToOpponentTrenchBalls

    private val path2 = if (shootFromProtected) {
        TrajectoryManager.opponentTrenchBallsToProtectedScoringLocation
    } else {
        TrajectoryManager.opponentTrenchBallsToInitLineScoringLocation
    }

    private val path3 = if (shootFromProtected) {
        TrajectoryManager.protectedScoringLocationToDoubleRendezvousPickup
    } else {
        TrajectoryManager.initLineScoringLocationToDoubleRendezvousPickup
    }

    private val path4 = TrajectoryManager.doubleRendezvousPickupToRendezvousIntermediate
    private val path5 = TrajectoryManager.rendezvousIntermediateToSingleRendezvousPickup

    private val path6 = if (shootFromProtected) {
        TrajectoryManager.singleRendezvousPickupToProtectedScoringLocation
    } else {
        TrajectoryManager.singleRendezvousPickupToInitLineScoringLocation
    }

    override fun getRoutine(): Command = sequential {
        +InstantCommand(Runnable { Drivetrain.resetPosition(WaypointManager.kStealStart) })
        +Drivetrain.followTrajectory(path1)
        +Drivetrain.followTrajectory(path2)
        +Drivetrain.followTrajectory(path3)
        +Drivetrain.followTrajectory(path4)
        +Drivetrain.followTrajectory(path5)
        +Drivetrain.followTrajectory(path6)
    }

    fun getDuration(): Double =
        path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds +
            path4.totalTimeSeconds + path5.totalTimeSeconds + path6.totalTimeSeconds
}
