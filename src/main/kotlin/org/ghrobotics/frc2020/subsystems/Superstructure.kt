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
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.ghrobotics.frc2020.TurretConstants
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.frc2020.planners.ShooterPlanner
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
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.derived.radians
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
    // Aiming and shooting parameters.
    private var latestAimingParameters = AimingParameters(SIUnit(0.0), SIUnit(0.0), SIUnit(0.0))
    private var latestShootingParameters = ShooterPlanner.ShooterParameters(SIUnit(0.0), SIUnit(0.0))

    // Hold angles and speeds.
    private var turretHoldAngle = 0.degrees
    private var shooterHoldSpeed = 0.radians / 1.seconds
    private var hoodHoldAngle = 0.degrees

    // Velocity threshold for drivetrain.
    private val kVelocityTreshold = 0.0.inches / 1.seconds

    // Amount of time required to stop.
    private val kStopTimeTreshold = 0.7.seconds

    // Error tolerance for the shooter.
    private val kShooterErrorTolerance = 360.degrees / 1.minutes * 225

    // Error tolerance for the hood.
    private val kHoodErrorTolerance = 1.degrees

    // Error tolerance for the turret.
    private val kTurretErrorTolerance = 1.degrees

    // Amount of time it takes to shoot 5 balls.
    private const val kShootTime = 10.0

    // Whether the turret is aligning or not.
    var visionAlign = false
        private set

    /**
     * Intakes power cells until the robot is stopped, then
     * shoot. This is primarily used for auto.
     */
    fun intakeUntilStoppedThenShoot() = object : SequentialCommandGroup() {
        init {
            addCommands(
                // Turn on LEDs
                InstantCommand(Runnable {
                    VisionProcessing.turnOnLEDs()
                    visionAlign = true
                }),

                // Intake and align until drivetrain stops.
                parallelDeadline(WaitForDrivetrainToStopCommand()) {
                    // Run and intake and feeder.
                    +IntakeCommand(1.0)
                    +AutoFeederCommand()

                    // Run turret, shooter, and hood.
                    +AutoTurretCommand { latestAimingParameters.turretAngleOuterGoal }
                    +AutoShooterCommand { latestShootingParameters.speed }
                    +AutoHoodCommand { latestShootingParameters.angle }
                },

                // Lock speeds and angles.
                InstantCommand(Runnable {
                    // Set hold angles and speeds.
                    turretHoldAngle = latestAimingParameters.turretAngleOuterGoal
                    shooterHoldSpeed = latestShootingParameters.speed
                    hoodHoldAngle = latestShootingParameters.angle

                    // Turn off LEDs.
                    VisionProcessing.turnOffLEDs()
                    visionAlign = false
                }),

                // Set speeds and angles, run feeder when ready.
                parallelDeadline(sequential {
                    +WaitUntilCommand {
//                        (Shooter.velocity - shooterHoldSpeed).absoluteValue < kShooterErrorTolerance &&
                            (Hood.angle - hoodHoldAngle).absoluteValue < kHoodErrorTolerance &&
                            (Turret.getAngle() - turretHoldAngle).absoluteValue < kTurretErrorTolerance
                    }
                    +ManualFeederCommand(1.0, 1.0).withTimeout(kShootTime)
                }) {
                    // Hold speeds and angles.
                    +AutoTurretCommand { turretHoldAngle }
                    +AutoShooterCommand { shooterHoldSpeed }
                    +AutoHoodCommand { hoodHoldAngle }
                }
            )
        }

        override fun end(interrupted: Boolean) {
            super.end(interrupted)
            VisionProcessing.turnOffLEDs()
            visionAlign = false
        }
    }

    val lockStart = Timer.getFPGATimestamp()
    val shooterPercent = 0.0

    /**
     * Aligns to the goal until the drivetrain has stopped, then
     * shoots power cells into the goal.
     */
    fun waitUntilStoppedThenShoot(feederTime: Double = kShootTime) = object : SequentialCommandGroup() {
        init {
            addCommands(
                // Turn on LEDs
                InstantCommand(Runnable {
                    VisionProcessing.turnOnLEDs()
                    visionAlign = true
                }),

                // Intake and align until drivetrain stops.
                parallelDeadline(WaitForDrivetrainToStopCommand()) {
                    // Run turret, shooter, and hood.
                    +AutoTurretCommand { latestAimingParameters.turretAngleOuterGoal }
                    +AutoShooterCommand { latestShootingParameters.speed }
                    +AutoHoodCommand { latestShootingParameters.angle }
                },

                // Lock speeds and angles.
                InstantCommand(Runnable {
                    // Set hold angles and speeds.
                    turretHoldAngle = latestAimingParameters.turretAngleOuterGoal
                    shooterHoldSpeed = latestShootingParameters.speed
                    hoodHoldAngle = latestShootingParameters.angle

                    // Turn off LEDs.
                    VisionProcessing.turnOffLEDs()
                    visionAlign = false
                }),

                // Set speeds and angles, run feeder when ready.
                parallelDeadline(sequential {
                    +WaitUntilCommand {
                        println("Shooter: " + (Shooter.velocity - shooterHoldSpeed).value * 60 / 6.28)
                        println("Hood: " + (Hood.angle - hoodHoldAngle).inDegrees())
                        println("Turret: " + (Turret.getAngle() - turretHoldAngle).inDegrees())

                        (Shooter.velocity - shooterHoldSpeed).absoluteValue < kShooterErrorTolerance &&
                            (Hood.angle - hoodHoldAngle).absoluteValue < kHoodErrorTolerance &&
                            ((Turret.getAngle() - turretHoldAngle).absoluteValue.value % (2 * Math.PI)) < kTurretErrorTolerance.value
                    }
                    +ManualFeederCommand(1.0, 1.0).withTimeout(feederTime)
                }) {
                    // Hold speeds and angles.
                    +AutoTurretCommand { turretHoldAngle }
                    +AutoShooterCommand { shooterHoldSpeed }
                    +AutoHoodCommand { hoodHoldAngle }
                }.withInterrupt {
                    System.out.printf("%2.2f, %4.2f, %4.2f %n",
                        Timer.getFPGATimestamp() - lockStart, shooterHoldSpeed.value, Shooter.velocity.value)
                    false
                }
            )
        }

        override fun end(interrupted: Boolean) {
            super.end(interrupted)
            VisionProcessing.turnOffLEDs()
            visionAlign = false
        }
    }

    /**
     * Intakes power cells.
     */
    fun intake(speed: Double = 0.75) = parallel {
        // Run the intake with a base speed of 0.5, scaling up linearly
        // to the robot's max speed.
        +IntakeCommand { speed }
        +AutoFeederCommand()
    }

    fun exhaust() = parallel {
        +IntakeCommand(-0.5)
        +ManualFeederCommand(-0.6, -0.75)
    }

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
            if (Drivetrain.averageVelocity.absoluteValue > kVelocityTreshold) {
                timer.reset()
            }
        }

        // End the command when the desired period has passed.

        override fun isFinished() = timer.hasPeriodPassed(kStopTimeTreshold.inSeconds())
    }
}
