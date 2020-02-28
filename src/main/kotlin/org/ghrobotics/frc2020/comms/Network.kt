/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.comms

import org.ghrobotics.frc2020.auto.Autonomous
import org.ghrobotics.frc2020.subsystems.Superstructure
import org.ghrobotics.frc2020.subsystems.climber.Climber
import org.ghrobotics.frc2020.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2020.subsystems.hood.Hood
import org.ghrobotics.frc2020.subsystems.shooter.Shooter
import org.ghrobotics.frc2020.subsystems.turret.Turret
import org.ghrobotics.lib.mathematics.twodim.geometry.x_u
import org.ghrobotics.lib.mathematics.twodim.geometry.y_u
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.inAmps
import org.ghrobotics.lib.mathematics.units.inFeet
import org.ghrobotics.lib.mathematics.units.inInches
import org.ghrobotics.lib.wrappers.networktables.enumSendableChooser
import org.ghrobotics.lib.wrappers.networktables.tab

/**
 * Handles all networking to Shuffleboard, including diagnostics from
 * various subsystems.
 */
object Network {

    // Sendable Chooser for auto mode.
    val autoModeSelector = enumSendableChooser<Autonomous.Mode>()

    init {
        tab("5190") {
            sendableChooser("Auto Mode", autoModeSelector) {
                position(row = 0, column = 0)
                size(width = 2, height = 2)
            }
            list("Robot Pose") {
                position(row = 0, column = 2)
                size(width = 2, height = 2)

                double("X (ft)") { Drivetrain.getPose().translation.x_u.inFeet() }
                double("Y (ft)") { Drivetrain.getPose().translation.y_u.inFeet() }
                double("Angle (Degrees)") { Drivetrain.getPose().rotation.degrees }
            }
            grid("Drivetrain") {
                position(row = 0, column = 4)
                size(width = 2, height = 2)

                double("Left Position (in)") { Drivetrain.leftPosition.inInches() }
                double("Right Position (in)") { Drivetrain.rightPosition.inInches() }
                double("Left Current (A)") { Drivetrain.leftCurrent.inAmps() }
                double("Right Current (A)") { Drivetrain.rightCurrent.inAmps() }
            }
            grid("Shooter") {
                position(row = 0, column = 6)
                size(width = 2, height = 1)

                double("RPM") { Shooter.velocity.value * 60 / 2 / Math.PI }
                double("Speed SI") { Shooter.velocity.value }
                double("Voltage (V)") { Shooter.voltage.value }
            }
            grid("Turret") {
                position(row = 0, column = 8)
                size(width = 2, height = 1)

                double("Angle (Degrees)") { Turret.getAngle().inDegrees() }
                double("Speed") { Math.toDegrees(Turret.speed.value) }
                double("Current (A)") { Turret.current.inAmps() }
            }
            grid("Hood") {
                position(row = 2, column = 0)
                size(width = 2, height = 1)

                double("Encoder Raw") { Hood.rawEncoder }
                double("Angle") { Hood.angle.inDegrees() }
            }
            grid("Climber") {
                position(row = 2, column = 2)
                size(width = 2, height = 1)

                double("Current (A)") { Climber.current.inAmps() }
            }
            grid("Superstructure") {
                position(row = 2, column = 4)
                size(width = 2, height = 2)

                double("Turret Hold Angle") { Superstructure.holdParams.turretAngle.inDegrees() }
                double("Shooter Hold Speed") { Superstructure.holdParams.shooterSpeed.value * 60 / 2 / Math.PI }
                double("Hood Hold Angle") { Superstructure.holdParams.hoodAngle.inDegrees() }
                double("Distance to Goal (in)") { Superstructure.holdParams.distance.inInches() }
            }
        }
    }
}
