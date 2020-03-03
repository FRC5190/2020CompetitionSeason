package org.ghrobotics.frc2020.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.frc2020.auto.AutoRoutine
import org.ghrobotics.frc2020.auto.TrajectoryManager
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

    }

    fun getDuration(): Double =
        path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds +
            path4.totalTimeSeconds + path5.totalTimeSeconds + path6.totalTimeSeconds

}