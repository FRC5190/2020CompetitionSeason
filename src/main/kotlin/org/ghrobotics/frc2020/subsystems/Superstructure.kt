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
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.ghrobotics.frc2020.planners.ShooterPlanner
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.feeder.FeederPercentCommand
import org.ghrobotics.frc2020.subsystems.feeder.FeederPositionCommand
import org.ghrobotics.frc2020.subsystems.hood.Hood
import org.ghrobotics.frc2020.subsystems.hood.HoodPositionCommand
import org.ghrobotics.frc2020.subsystems.intake.IntakePercentCommand
import org.ghrobotics.frc2020.subsystems.shooter.Shooter
import org.ghrobotics.frc2020.subsystems.shooter.ShooterVelocityCommand
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.frc2020.subsystems.turret.TurretPositionCommand
import org.ghrobotics.frc2020.vision.GoalTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.rpm
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
    private const val kDefaultIntakeSpeed = 0.9

    private val kShooterTolerance = 30.rpm
    private val kHoodTolerance = 0.8.degrees

    // Locking parameters
    private var lockedShooterParams = ShooterPlanner.ShooterParameters(SIUnit(0.0), SIUnit(0.0), 0.0)
    private var lockedTurretAngle = 0.degrees

    // States
    var isAiming = false
        private set

    var ready = false
        private set

    /**
     * Intakes power cells from the ground.
     */
    fun intake(intakeSpeed: Double = kDefaultIntakeSpeed): Command = parallel {
        +IntakePercentCommand(intakeSpeed)
        +FeederPositionCommand()
    }

    /**
     * Releases power cells through the intake.
     */
    fun release(): Command = parallel {
        +IntakePercentCommand(-kDefaultIntakeSpeed)
        +FeederPercentCommand(-0.6, -0.6)
    }

    /**
     * Scores power cells into the high goal once the robot comes
     * to a stop. This method uses a fixed distance to the goal, and is
     * therefore helpful for autonomous mode.
     */
    fun scoreWhenStopped(distance: SIUnit<Meter>): Command =
        object : ParallelCommandGroup() {
            // Get the shooter parameters for this distance.
            val shooterParams = ShooterPlanner[distance]

            init {
                addCommands(
                    // Set the isAiming flag.
                    InstantCommand(Runnable { isAiming = true }),

                    // Spin up shooter and move hood to desired angle.
                    ShooterVelocityCommand(shooterParams.speed),
                    HoodPositionCommand(shooterParams.angle),

                    // Wait until drivetrain is stopped, shooter is at reference,
                    // and hood is at reference; then fire.
                    sequential {
                        +parallel {
                            +WaitForDrivetrainToStopCommand()
                            +WaitUntilCommand {
                                (Shooter.velocity - shooterParams.speed).absoluteValue < kShooterTolerance &&
                                    (Hood.angle - shooterParams.angle).absoluteValue < kHoodTolerance
                            }
                        }.deadlineWith(TurretPositionCommand(Turret.defaultBehavior))

                        // Store locked turret angle.
                        +InstantCommand(Runnable { lockedTurretAngle = Turret.getAngle() })

                        // Feed balls.
                        +parallel {
                            +TurretPositionCommand { lockedTurretAngle }
                            +FeederPercentCommand(lockedShooterParams.feedRate, 0.8)
                        }
                    }
                )
            }

            override fun end(interrupted: Boolean) {
                super.end(interrupted)
                isAiming = false
            }
        }

    /**
     * Scores power cells into the high goal once the robot comes
     * to a stop.
     */
    fun scoreWhenStopped(): Command =
        object : ParallelCommandGroup() {
            init {
                addCommands(
                    // Set the isAiming flag.
                    InstantCommand(Runnable { isAiming = true }),

                    // Spin and shooter and move hood to desired angle.
                    ShooterVelocityCommand {
                        if (!ready) ShooterPlanner[GoalTracker.latestTurretToGoalDistance].speed else
                            lockedShooterParams.speed
                    },
                    HoodPositionCommand {
                        if (!ready) ShooterPlanner[GoalTracker.latestTurretToGoalDistance].angle else
                            lockedShooterParams.angle
                    },

                    // Wait until drivetrain is stopped, shooter is at reference,
                    // and hood is at reference; then fire.
                    sequential {
                        +WaitForDrivetrainToStopCommand().deadlineWith(TurretPositionCommand(Turret.defaultBehavior))
                        +parallel {
                            +InstantCommand(Runnable {
                                lockedShooterParams = ShooterPlanner[GoalTracker.latestTurretToGoalDistance]
                                ready = true
                            })
                            +WaitUntilCommand {
                                println((Shooter.velocity - lockedShooterParams.speed).value / 2 / Math.PI * 60)
                                (Shooter.velocity - lockedShooterParams.speed).absoluteValue < kShooterTolerance &&
                                    (Hood.angle - lockedShooterParams.angle).absoluteValue < kHoodTolerance

                            }
                        }.deadlineWith(TurretPositionCommand(Turret.defaultBehavior))

                        // Store locked turret angle.
                        +InstantCommand(Runnable { lockedTurretAngle = Turret.getAngle() })

                        // Feed balls.
                        +parallel {
                            +TurretPositionCommand { lockedTurretAngle }
                            +FeederPercentCommand({ lockedShooterParams.feedRate }, { 0.8 })
                        }
                    }
                )
            }

            override fun end(interrupted: Boolean) {
                super.end(interrupted)
                isAiming = false
                ready = false
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
            private val kStopTimeThreshold = 0.4.seconds
        }
    }
}
