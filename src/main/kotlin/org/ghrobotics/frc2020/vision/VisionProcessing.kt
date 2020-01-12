package org.ghrobotics.frc2020.vision

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DigitalOutput

object VisionProcessing {
    val camera = ChameleonCamera("USB Camera-B4.09.24.1")
    val angle get() = camera.yaw

    val light = DigitalOutput(0)

    fun turnOnLight() {
        light.set(false)
    }

    fun turnOffLight() {
        light.set(true)
    }
}