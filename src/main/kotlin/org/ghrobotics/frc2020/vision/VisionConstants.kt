/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.vision

import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Transform2d
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.seconds

object VisionConstants {
    // Turret to camera transform
    val kTurretToCamera = Transform2d(10.5.inches, 0.inches, Rotation2d.fromDegrees(0.8))

    // TargetTracker constants
    val kMaxTargetTrackingLifetime = 1.5.seconds
    val kTargetTrackingDistanceErrorTolerance = 25.inches
    const val kMedianWindowSize = 15
}
