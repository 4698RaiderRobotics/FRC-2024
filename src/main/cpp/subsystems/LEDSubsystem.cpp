// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDSubsystem.h"

LEDSubsystem::LEDSubsystem() {
    ctre::phoenix::led::CANdleConfiguration config;
    config.stripType = ctre::phoenix::led::LEDStripType::RGB;
    m_leds.ConfigAllSettings(config);
};

// This method will be called once per scheduler run
void LEDSubsystem::Periodic() {

}

void LEDSubsystem::SetColor(int r, int g, int b) {
    m_leds.SetLEDs(r, g, b, 0, 8, 31);
}