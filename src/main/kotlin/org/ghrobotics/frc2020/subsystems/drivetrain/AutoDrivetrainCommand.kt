package org.ghrobotics.frc2020.subsystems.drivetrain

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.LinearVelocity

/**
 * Sets the velocity of the two sides of the drivetrain.
 *
 * @param left The desired left velocity.
 * @param right The desired right velocity.
 */
class AutoDrivetrainCommand(
    private val left: SIUnit<LinearVelocity>,
    private val right: SIUnit<LinearVelocity>
) : FalconCommand(Drivetrain) {
    override fun initialize() {
        Drivetrain.setOutputSI(left.value, right.value, 0.0, 0.0)
    }
}