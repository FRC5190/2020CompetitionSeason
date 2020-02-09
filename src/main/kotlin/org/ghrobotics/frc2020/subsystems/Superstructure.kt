/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.ghrobotics.frc2020.TurretConstants
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.frc2020.planners.ShooterPlanner
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.feeder.ManualFeederCommand
import org.ghrobotics.frc2020.subsystems.hood.AutoHoodCommand
import org.ghrobotics.frc2020.subsystems.intake.IntakeCommand
import org.ghrobotics.frc2020.subsystems.shooter.AutoShooterCommand
import org.ghrobotics.frc2020.subsystems.turret.AutoTurretCommand
import org.ghrobotics.frc2020.vision.GoalTracker
import org.ghrobotics.frc2020.vision.VisionProcessing
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.inSeconds
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.minutes
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds

/**
 * Represents the overall superstructure of the robot, including the turret,
 * shooter, intake, and climbing mechanisms.
 */
object Superstructure {
    // Latest aiming parameters for the superstructure.
    private var latestAimingParameters = AimingParameters(Rotation2d(), SIUnit(0.0))

    // Latest shooting parameters for the hood and the shooter.
    private var latestShootingParameters = ShooterPlanner.ShooterParameters(SIUnit(0.0), SIUnit(0.0))

    // Variables to store speeds and angles.
    private var turretHoldAngle = 0.degrees
    private var shooterHoldSpeed = 0.radians / 1.seconds
    private var hoodHoldAngle = 0.degrees

    private val kVelocityTreshold = 0.25.inches / 1.seconds
    private val kStopTimeTreshold = 0.6.seconds

    // Positions
    private val kTurretStowedPosition = 90.degrees

    /**
     * Shoots power cells into the goal.
     *
     * @return The command.
     */
    fun shootPowerCells(): Command = object : SequentialCommandGroup() {
        init {
            addCommands(
                // Turn on LEDs so that we can start aiming.
                InstantCommand(Runnable { VisionProcessing.turnOnLEDs() }),

                // Aim turret, shooter, and hood until the drivetrain is stopped for
                // at least 0.7 seconds.
                parallelDeadline(
                    object : FalconCommand() {
                        val timer = Timer()

                        override fun initialize() = timer.start()
                        override fun execute() {
                            if (Drivetrain.averageVelocity > kVelocityTreshold) {
                                timer.reset()
                            }
                        }

                        override fun isFinished() = timer.hasPeriodPassed(kStopTimeTreshold.inSeconds())
                    }
                ) {
                    // Turret Alignment
                    +sequential {
                        // Add a slight delay so that we can register the target.
                        +WaitCommand(0.2)
                        // Turn the turret.
                        +AutoTurretCommand { SIUnit(latestAimingParameters.turretAngle.radians) }
                    }
                    // Shooter and Hood
                    +AutoShooterCommand { latestShootingParameters.speed }
                    +AutoHoodCommand { latestShootingParameters.angle }
                },

                // Store the current values for relevant subsystems and turn off camera.
                InstantCommand(Runnable {
                    VisionProcessing.turnOffLEDs()
                    turretHoldAngle = SIUnit(latestAimingParameters.turretAngle.radians)
                    shooterHoldSpeed = latestShootingParameters.speed
                    hoodHoldAngle = latestShootingParameters.angle
                }),

                parallel {
                    // Respective subsystems should hold state.
                    +AutoTurretCommand { turretHoldAngle }
                    +AutoShooterCommand { shooterHoldSpeed }
                    +AutoHoodCommand { hoodHoldAngle }

                    // Feed balls
                    +ManualFeederCommand { 0.3 }
                }
            )
        }

        override fun end(interrupted: Boolean) {
            VisionProcessing.turnOffLEDs()
        }
    }

    fun intake() = parallel {
        +IntakeCommand({ -0.5 }, { 0.7 })
        +ManualFeederCommand { 0.6 }
    }

    fun exhaust() = parallel {
        +IntakeCommand({ -0.5 }, { -0.7 })
        +ManualFeederCommand { -0.6 }
    }

    fun shoot() = parallel {
        +AutoShooterCommand { 360.degrees / 1.minutes * 4000}
        +sequential {
            +WaitCommand(2.0)
            +ManualFeederCommand { 0.9 }
        }
    }

    /**
     * Goes to a safe stowed position.
     */
    fun goToStowedPosition() = AutoTurretCommand { kTurretStowedPosition }

    /**
     * Returns the latest aiming parameters for the shooter and the turret.
     *
     * @return The latest aiming parameters for the shooter and the turret.
     */
    private fun getAimingParameters(): AimingParameters {
        // Get the predicted field-relative robot pose.
        val robotPose = Drivetrain.getPredictedPose(TurretConstants.kAlignDelay)

        // Get the turret pose, assuming the turret is locked to 0 degrees.
        val turretPose = robotPose + Transform2d(TurretConstants.kTurretRelativeToRobotCenter, Rotation2d())

        // Get the target that is closest to the turret pose.
        val target = GoalTracker.getClosestTarget(turretPose)

        val turretToGoal = if (target != null) {
            // Get the goal's pose in turret's coordinates.
            target.averagePose.relativeTo(turretPose)
        } else {
            // Get an approximation from the current robot pose and the known target pose.
            VisionConstants.kGoalLocation.relativeTo(turretPose)
        }

        // Get the distance to the target.
        val distance = turretToGoal.translation.norm

        // Get the angle to the target.
        val angle = Rotation2d(turretToGoal.translation.x, turretToGoal.translation.y)

        // Return the distance and angle.
        return AimingParameters(angle, SIUnit(distance))
    }

    /**
     * Updates all superstructure states.
     */
    fun update() {
        latestAimingParameters = getAimingParameters()
        latestShootingParameters = ShooterPlanner[latestAimingParameters.distance]
    }

    /**
     * Aiming parameters for the shooter and the turret.
     */
    data class AimingParameters(
        val turretAngle: Rotation2d,
        val distance: SIUnit<Meter>
    )
}
