package org.ghrobotics.frc2020.subsystems.climber

import org.ghrobotics.lib.commands.FalconCommand

/**
 * Extends or retracts the climber assembly.
 *
 * @param extend Whether to extend or retract the climber assembly.
 */
class ExtendClimberCommand(private val extend: Boolean) : FalconCommand(Climber) {
    override fun initialize() {
        Climber.extend(extend)
    }

    override fun isFinished(): Boolean {
        return true
    }
}