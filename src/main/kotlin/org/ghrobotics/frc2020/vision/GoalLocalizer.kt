/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.vision

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import org.ghrobotics.frc2020.VisionConstants
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.utils.toTransform

/**
 * Finds the robot position relative to the goal using solvePnP.
 */
object GoalLocalizer {
    // Location of the goal in the field-coordinate system.
    private val kGoalLocation = Pose2d(54.feet, 94.66.inches, Rotation2d())

    /**
     * Calculates the robot pose relative to the goal using solvePnP vision
     * data only.
     *
     * @param cameraToGoal The camera to goal transformation.
     * @return The robot pose.
     */
    fun calculateRobotPose(cameraToGoal: Transform2d): Pose2d {
        // We will first calculate the goal relative to the robot's coordinates.
        val turretToGoal = VisionConstants.kTurretToCamera + cameraToGoal
        val robotToGoal = Turret.robotToTurret + turretToGoal.toTransform()

        // We can simply invert the transform to calculate the robot in the goal's coordinates.
        val goalToRobot = -robotToGoal.toTransform()

        // Now find the robot pose in the coordinates of the goal.
        return kGoalLocation + goalToRobot
    }

    /**
     * Calculates the robot pose relative to the goal using solvePnp vision
     * data for translation and the gyro angle for the rotational component.
     *
     * @param cameraToGoal The camera to goal transformation.
     * @param robotAngle The angle of the robot (relative to the field).
     *
     * @return The robot pose.
     */
    fun calculateRobotPose(cameraToGoal: Transform2d, robotAngle: Rotation2d): Pose2d {
        // Get the approximate robot pose from the camera alone.
        val approxRobotPose = calculateRobotPose(cameraToGoal)

        // Use the provided robot rotation and return it.
        return Pose2d(approxRobotPose.translation, robotAngle)
    }

    private operator fun Transform2d.unaryMinus() =
        Transform2d(-translation.rotateBy(-rotation), -rotation)
}
