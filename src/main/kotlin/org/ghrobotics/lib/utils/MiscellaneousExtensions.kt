/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.utils

import edu.wpi.first.wpilibj.geometry.Pose2d

/**
 * Converts a Pose2d to a Transform2d.
 */
fun Pose2d.toTransform() = minus(Pose2d())
