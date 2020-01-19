/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020

import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.minutes
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.operations.times
import org.ghrobotics.lib.mathematics.units.specialops.times

/**
 * Contains constants for the drivetrain.
 */
@Suppress("MemberVisibilityCanBePrivate", "unused")
object DriveConstants {
    const val kLeftMasterId = 1
    const val kLeftSlave1Id = 2
    const val kRightMasterId = 3
    const val kRightSlave1Id = 4

    const val kPigeonId = 17

    val kWheelRadius = 3.inches
    val kTrackWidth = 27.75.inches
    val kNativeUnitModel = NativeUnitLengthModel(9.09.nativeUnits, kWheelRadius)

    val kP = 0.0
}

object IntakeConstants {
    const val kIntakeId = 9
}

object FortuneWheelConstants {
    // Determines the bit-depth of colors when being compared.
    // The bit-depth determines the amount of possible colors.
    // Ex: '255' would result in an 8bit color spectrum.
    const val kColorBitDepth = 9

    // Spinner
    const val kSpinnerSpeed = 0.2
    const val kSpinnerMotorId = 8
    val kSpinnerRadius = 3.inches // Radius of wheel connected to the spinner motor
    val kSpinnerUnitModel = NativeUnitLengthModel(42.nativeUnits, kSpinnerRadius) // Unit model for the spinner motor

    // Accuracy
    val kDataAccuracy = 4
    val kCompletion = 10
}
