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
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.ghrobotics.frc2020.TurretConstants
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.frc2020.planners.ShooterPlanner
import org.ghrobotics.frc2020.subsystems.drivetrain.AutoDrivetrainCommand
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.feeder.AutoFeederCommand
import org.ghrobotics.frc2020.subsystems.feeder.ManualFeederCommand
import org.ghrobotics.frc2020.subsystems.hood.AutoHoodCommand
import org.ghrobotics.frc2020.subsystems.hood.Hood
import org.ghrobotics.frc2020.subsystems.intake.IntakeCommand
import org.ghrobotics.frc2020.subsystems.shooter.AutoShooterCommand
import org.ghrobotics.frc2020.subsystems.shooter.Shooter
import org.ghrobotics.frc2020.subsystems.turret.AutoTurretCommand
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.frc2020.vision.GoalTracker
import org.ghrobotics.frc2020.vision.VisionProcessing
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.inSeconds
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.minutes
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.mathematics.units.unitlessValue

/**
 * Represents the overall superstructure of the robot, including the turret,
 * shooter, intake, and climbing mechanisms.
 */
object Superstructure {
    private var latestAimingParameters = AimingParameters(SIUnit(0.0), SIUnit(0.0), SIUnit(0.0))
    private var latestShootingParameters = ShooterPlanner.ShooterParameters(SIUnit(0.0), SIUnit(0.0))

    private var turretHoldAngle = 0.degrees
    private var shooterHoldSpeed = 0.radians / 1.seconds
    private var hoodHoldAngle = 0.degrees

    private val kVelocityTreshold = 0.25.inches / 1.seconds
    private val kStopTimeTreshold = 0.7.seconds

    private val kShooterErrorTolerance = 360.degrees / 1.minutes * 20
    private val kHoodErrorTolerance = 1.degrees
    private val kTurretErrorTolerance = 1.degrees

    private val kTurretStowedPosition = 90.degrees

    /**
     * Aims the superstructure to the hexagonal goal. This command
     * will end when the drivetrain has stopped and all subsystems
     * have finished alignment. Once the drivetrain has stopped, it
     * will be locked in place.
     */
    fun aimToGoal(): Command {
        return sequential {
            // Turn on LEDs for targeting.
            +InstantCommand(Runnable { VisionProcessing.turnOnLEDs() })

            // Turn the turret to roughly the goal location.
            +AutoTurretCommand.createFromFieldOrientedAngle(VisionConstants.kGoalFieldRelativeAngle)
                .withInterrupt { Turret.getFieldRelativeAngle() in (-75).degrees..75.degrees }

            // Start aiming the turret at the goal, spin up the shooter, and set
            // hood angle until the drivetrain has stopped.
            +parallelDeadline(WaitForDrivetrainToStopCommand()) {
                // Align the turret.
                +AutoTurretCommand(latestAimingParameters::turretAngleOuterGoal)

                // Align the shooter.
                +AutoShooterCommand(latestShootingParameters::speed)

                // Align the hood.
                +AutoHoodCommand(latestShootingParameters::angle)
            }

            // Store the setpoints.
            +InstantCommand(Runnable {
                // Set hold angles and speeds.
                turretHoldAngle = latestAimingParameters.turretAngleOuterGoal
                shooterHoldSpeed = latestShootingParameters.speed
                hoodHoldAngle = latestShootingParameters.angle

                // Turn off LEDs.
                VisionProcessing.turnOffLEDs()
            })

            // Set setpoints and lock drivetrain.
            +parallel {
                // Setpoints.
                +AutoTurretCommand { turretHoldAngle }
                +AutoShooterCommand { shooterHoldSpeed }
                +AutoHoodCommand { hoodHoldAngle }

                // Drive Lock.
                +ScheduleCommand(AutoDrivetrainCommand(SIUnit(0.0), SIUnit(0.0)))
            }.withInterrupt {
                // Exit when within tolerances.
                (Shooter.velocity - shooterHoldSpeed).absoluteValue < kShooterErrorTolerance &&
                    (Hood.angle - hoodHoldAngle).absoluteValue < kHoodErrorTolerance &&
                    (Turret.getAngle() - turretHoldAngle).absoluteValue < kTurretErrorTolerance
            }
        }
    }

    /**
     * Shoots the power cells after aiming into the goal. The aim
     * command must be called before this one because it sets the hold
     * angles and speeds. This command does not end unless interrupted
     * manually.
     */
    fun shootIntoGoal(): Command {
        return parallel {
            // Set setpoints.
            +AutoTurretCommand { turretHoldAngle }
            +AutoShooterCommand { shooterHoldSpeed }
            +AutoHoodCommand { hoodHoldAngle }

            // Drive Lock.
            +AutoDrivetrainCommand(SIUnit(0.0), SIUnit(0.0))

            // Feed balls when ready.
            +sequential {
                +WaitUntilCommand {
                    (Shooter.velocity - shooterHoldSpeed).absoluteValue < kShooterErrorTolerance &&
                        (Hood.angle - hoodHoldAngle).absoluteValue < kHoodErrorTolerance &&
                        (Turret.getAngle() - turretHoldAngle).absoluteValue < kTurretErrorTolerance
                }
                +ManualFeederCommand(0.9, 0.9)
            }
        }
    }

    /**
     * Intakes power cells.
     */
    fun intake() = parallel {
        // Run the intake with a base speed of 0.5, scaling up linearly
        // to the robot's max speed.
        +IntakeCommand { 0.5 + (1.0 - 0.5) * (Drivetrain.averageVelocity / Drivetrain.kMaxSpeed).unitlessValue }
        +AutoFeederCommand()
    }

    fun exhaust() = parallel {
        +IntakeCommand(-0.5)
        +ManualFeederCommand(-0.6, -0.75)
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

        val (turretToOuterGoal, turretToInnerGoal) = if (target != null) {
            // Get the goal's pose in turret's coordinates.
            target.averagePose.relativeTo(turretPose) to
                (target.averagePose + VisionConstants.kOuterToInnerGoalTransform).relativeTo(turretPose)
        } else {
            // Get an approximation from the current robot pose and the known target pose.
            VisionConstants.kGoalLocation.relativeTo(turretPose) to
                (VisionConstants.kGoalLocation + VisionConstants.kOuterToInnerGoalTransform).relativeTo(turretPose)
        }

        // Get the distance to the target.
        val distance = turretToInnerGoal.translation.norm

        // Get the angle to the target.
        val angleOuter = Rotation2d(turretToOuterGoal.translation.x, turretToOuterGoal.translation.y)
        val angleInner = Rotation2d(turretToInnerGoal.translation.x, turretToInnerGoal.translation.y)

        // Return the distance and angle.
        return AimingParameters(angleOuter.toSI(), angleInner.toSI(), SIUnit(distance))
    }

    /**
     * Updates all superstructure states.
     */
    fun update() {
        latestAimingParameters = getAimingParameters()
        latestShootingParameters = ShooterPlanner[latestAimingParameters.distance]
    }

    private fun Rotation2d.toSI() = SIUnit<Radian>(radians)

    /**
     * Aiming parameters for the shooter and the turret.
     */
    data class AimingParameters(
        val turretAngleOuterGoal: SIUnit<Radian>,
        val turretAngleInnerGoal: SIUnit<Radian>,
        val distance: SIUnit<Meter>
    )

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
            if (Drivetrain.averageVelocity > kVelocityTreshold) {
                timer.reset()
            }
        }

        // End the command when the desired period has passed.
        override fun isFinished() = timer.hasPeriodPassed(kStopTimeTreshold.inSeconds())
    }
}
