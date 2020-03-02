/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.ghrobotics.frc2020.planners.ShooterPlanner
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.feeder.FeederPercentCommand
import org.ghrobotics.frc2020.subsystems.hood.Hood
import org.ghrobotics.frc2020.subsystems.hood.HoodPositionCommand
import org.ghrobotics.frc2020.subsystems.shooter.Shooter
import org.ghrobotics.frc2020.subsystems.shooter.ShooterVelocityCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.inSeconds
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds

/**
 * Represents the overall superstructure of the robot, including the turret,
 * shooter, intake, and climbing mechanisms.
 */
object Superstructure {

    // Constants
    private const val kDefaultFeedRate = 0.70
    private const val kMaxFeedRateShootingTime = 1.5

    private val kShooterTolerance = 10.radians / 1.seconds
    private val kHoodTolerance = 0.5.degrees

    /**
     * Scores power cells into the high goal once the robot comes
     * to a stop. This method uses a fixed distance to the goal, and is
     * therefore helpful for autonomous mode.
     */
    fun scoreWhenStopped(distance: SIUnit<Meter>, feedRate: Double = kDefaultFeedRate): Command = parallel {
        // Get the shooter parameters for this distance.
        val shooterParams = ShooterPlanner[distance]

        // Spin up shooter and move hood to desired angle.
        +ShooterVelocityCommand(shooterParams.speed)
        +HoodPositionCommand(shooterParams.angle)

        // Wait until drivetrain is stopped, shooter is at reference,
        // and hood is at reference; then fire.
        +sequential {
            +parallel {
                +WaitForDrivetrainToStopCommand()
                +WaitUntilCommand {
                    (Shooter.velocity - shooterParams.speed).absoluteValue < kShooterTolerance &&
                        (Hood.angle - shooterParams.angle).absoluteValue < kHoodTolerance
                }
            }

            // Feed balls.
            +FeederPercentCommand(feedRate, 0.8).withTimeout(kMaxFeedRateShootingTime * feedRate)
        }
    }

    /**
     * A command that does not exit until the drivetrain has stopped
     * for a certain period of time.
     */
    private class WaitForDrivetrainToStopCommand : FalconCommand() {

        // Timer to keep track of the time the drivetrain has stopped.
        val timer = Timer()

        // Start the timer.
        override fun initialize() = timer.start()

        // Check the drivetrain's velocity.
        override fun execute() {
            if (Drivetrain.averageVelocity.absoluteValue > kVelocityThreshold) {
                timer.reset()
            }
        }

        // End the command when the desired period has passed.
        override fun isFinished() = timer.hasPeriodPassed(kStopTimeThreshold.inSeconds())

        companion object {
            private val kVelocityThreshold = 0.1.inches / 1.seconds
            private val kStopTimeThreshold = 0.2.seconds
        }
    }
}
