package org.ghrobotics.lib.utils

import edu.wpi.first.wpilibj.geometry.Pose2d

/**
 * Converts a Pose2d to a Transform2d.
 */
fun Pose2d.toTransform() = minus(Pose2d())