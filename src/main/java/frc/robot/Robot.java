// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands;
/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final double MAX_SPEED = 0.25;
  private static final double MIN_SPEED = -0.25;

  private static final int kMotorPort = 13;
  private static final int kJoystickPort = 0;

  private final SparkFlex m_motor;
  private final XboxController m_joystick;
  private final RelativeEncoder m_encoder;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_motor = new SparkFlex(kMotorPort, MotorType.kBrushless);
    m_joystick = new XboxController(kJoystickPort);
    m_encoder = m_motor.getEncoder();
    SmartDashboard.putData("Check", systemCheckCommand());


  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder", m_encoder.getPosition());
    SmartDashboard.putNumber("Joystick", m_joystick.getLeftX());
    SmartDashboard.putNumber("Motor", m_motor.getAppliedOutput());
  }
  public void registerSystemCheckWithSmartDashboard() {
    Command systemCheck = systemCheckCommand();
  
  }
  /** The teleop periodic function is called every control packet in teleop. */
  @Override
  public void teleopPeriodic() {
    double voltagePercentage = MathUtil.applyDeadband(m_joystick.getLeftX(), .1);
    voltagePercentage = MathUtil.clamp(voltagePercentage, MIN_SPEED, MAX_SPEED);

  m_motor.set(m_joystick.getLeftX());
  }
  public void runMotor() {
    m_motor.set(.1);
  }
  public void stopMotor() {
    m_motor.set(0.0);
  }
  protected Command systemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
                runMotor(); // Start the motor
            }
        ),
        Commands.waitSeconds(3.0), // Wait for 3 seconds
        Commands.runOnce(
            () -> {
                stopMotor(); // Stop the motor
            }
        )
    );
          }}
          