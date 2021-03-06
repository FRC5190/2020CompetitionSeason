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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
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
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.rpm
import org.ghrobotics.lib.mathematics.units.inSeconds
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.utils.Source

/**
 * Represents the overall superstructure of the robot, including the turret,
 * shooter, intake, and climbing mechanisms.
 */
object Superstructure {

    // Constants
    private const val kDefaultIntakeSpeed = 1.0

    private val kShooterTolerance = 30.rpm
    private val kHoodTolerance = 1.degrees

    // Locking parameters
    private var lockedShooterParams = ShooterPlanner.ShooterParameters(SIUnit(0.0), SIUnit(0.0), 0.0)
    private var lockedTurretAngle = 0.degrees

    // States
    var isAiming = false
        private set

    var readyToFire = false
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
    fun scoreWhenStopped(
        distance: SIUnit<Meter>,
        readyToFire: Source<Boolean> = { true },
        feedTime: Double = 5.0
    ): Command =
        object : ParallelDeadlineGroup(
            // Wait until drivetrain is stopped, shooter is at reference,
            // and hood is at reference; then fire.
            sequential {
                +InstantCommand(Runnable { lockedShooterParams = ShooterPlanner[distance] })
                +WaitForDrivetrainToStopCommand().deadlineWith(TurretPositionCommand(Turret.defaultBehavior))
                +parallel {
                    +WaitUntilCommand {
                        (Shooter.velocity - lockedShooterParams.speed).absoluteValue < kShooterTolerance &&
                            (Hood.angle - lockedShooterParams.angle).absoluteValue < kHoodTolerance
                    }.withTimeout(0.01)
                    +WaitUntilCommand(readyToFire)
                }.deadlineWith(TurretPositionCommand(Turret.defaultBehavior))

                // Store locked turret angle.
                +InstantCommand(Runnable { lockedTurretAngle = GoalTracker.latestTurretAngleToFaceInnerGoal })

                // Feed balls.
                +parallelDeadline(
                    FeederPercentCommand(
                        { lockedShooterParams.feedRate },
                        { 0.8 }).withTimeout(feedTime)
                ) {
                    +TurretPositionCommand { lockedTurretAngle }
                }
            }
        ) {
            // Get the shooter parameters for this distance.

            init {
                addCommands(
                    // Set the isAiming flag.
                    InstantCommand(Runnable { isAiming = true }),

                    // Spin up shooter and move hood to desired angle.
                    ShooterVelocityCommand { lockedShooterParams.speed },
                    HoodPositionCommand { lockedShooterParams.angle }
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
    fun scoreWhenStopped(
        readyToFire: Source<Boolean> = { true },
        feedTime: Double = 5.0,
        aimToInner: Boolean = false
    ): Command =
        object : ParallelDeadlineGroup(
            // Wait until drivetrain is stopped, shooter is at reference,
            // and hood is at reference; then fire.
            sequential {
                +WaitForDrivetrainToStopCommand().deadlineWith(TurretPositionCommand(if (aimToInner) Turret.innerGoalBehavior else Turret.defaultBehavior))
                +parallel {
                    +InstantCommand(Runnable {
                        lockedShooterParams = ShooterPlanner[GoalTracker.latestTurretToGoalDistance]
                        this@Superstructure.readyToFire = true
                    })
                    +WaitUntilCommand {
                        (Shooter.velocity - lockedShooterParams.speed).absoluteValue < kShooterTolerance &&
                            (Hood.angle - lockedShooterParams.angle).absoluteValue < kHoodTolerance
                    }
                    +WaitUntilCommand(readyToFire)
                }.deadlineWith(TurretPositionCommand(if (aimToInner) Turret.innerGoalBehavior else Turret.defaultBehavior))

                // Store locked turret angle.
                +InstantCommand(Runnable {
                    lockedTurretAngle =
                        if (aimToInner) GoalTracker.latestTurretAngleToFaceInnerGoal else GoalTracker.latestTurretAngleToFaceOuterGoal
                })

                // Feed balls.
                +parallelDeadline(
                    FeederPercentCommand({ lockedShooterParams.feedRate }, { 0.8 }).withTimeout(feedTime)
                ) {
                    +TurretPositionCommand { lockedTurretAngle }
                }
            }
        ) {
            init {
                addCommands(
                    // Set the isAiming flag.
                    InstantCommand(Runnable { isAiming = true }),

                    // Spin and shooter and move hood to desired angle.
                    ShooterVelocityCommand {
                        if (!this@Superstructure.readyToFire) ShooterPlanner[GoalTracker.latestTurretToGoalDistance].speed else
                            lockedShooterParams.speed
                    },
                    HoodPositionCommand {
                        if (!this@Superstructure.readyToFire) ShooterPlanner[GoalTracker.latestTurretToGoalDistance].angle else
                            lockedShooterParams.angle
                    }
                )
            }

            override fun end(interrupted: Boolean) {
                super.end(interrupted)
                isAiming = false
                this@Superstructure.readyToFire = false
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
            private val kVelocityThreshold = 2.5.inches / 1.seconds
            private val kStopTimeThreshold = 0.2.seconds
        }
    }
}
