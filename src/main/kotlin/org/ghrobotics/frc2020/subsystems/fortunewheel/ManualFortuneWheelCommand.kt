package org.ghrobotics.frc2020.subsystems.fortunewheel

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

class ManualFortuneWheelCommand(val speed: DoubleSource) : FalconCommand(FortuneWheel) {
    override fun execute() {
        var color = FortuneWheel.rawColor
        FortuneWheel.setPercent(speed() / 5)
        println("Red: " + color.red + " | Green: " + color.green + " | Blue: " + color.blue)
    }
}