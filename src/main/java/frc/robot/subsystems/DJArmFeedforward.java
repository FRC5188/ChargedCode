// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// This code is taken from windham windup here: https://github.com/WHS-FRC-3467/Skip-5.13/tree/master/src/main/java/frc/robot/subsystems/arm
// They copied from https://github.com/Mechanical-Advantage/RobotCode2023/tree/main/src/main/java/org/littletonrobotics/frc2023/subsystems/arm
// both of these teams have awesome build threads for 2023 and are a great help!

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;



/**
 * Note: we currently have the inverse and forward kinematic functions in our arm subsystem class. We might want to do what they did 
 * and move it to its on class. We might want to copy their math too. but ours seems to be working. Garrett 2/21/23
 */

/**
 * Calculates feedforward voltages for a double jointed arm.
 *
 * <p>https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 * Adapted from 6328 ArmDynamics class
 */
public class DJArmFeedforward {
  private static final double g = 9.80665;
  
  private final JointConfig Joint_Lower;
  private final JointConfig Joint_Upper;

  public DJArmFeedforward(JointConfig Joint_Lower, JointConfig Joint_Upper) {
    this.Joint_Lower = Joint_Lower;
    this.Joint_Upper = Joint_Upper;
  }

  /** Calculates the joint voltages based on the joint positions (feedforward). */
  public Vector<N2> feedforward(Vector<N2> position) {
    return feedforward(position, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0, 0.0));
  }

  public Vector<N2> feedforward(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
    var torque =
        M(position)
            .times(acceleration)
            .plus(C(position, velocity).times(velocity))
            .plus(Tg(position));
    return VecBuilder.fill(
        Joint_Lower.motor.getVoltage(torque.get(0, 0), velocity.get(0, 0)),
        Joint_Upper.motor.getVoltage(torque.get(1, 0), velocity.get(1, 0)));
  }


  private Matrix<N2, N2> M(Vector<N2> position) {
    var M = new Matrix<>(N2.instance, N2.instance);
    M.set(
        0,
        0,
        Joint_Lower.mass * Math.pow(Joint_Lower.cgRadius, 2.0)
            + Joint_Upper.mass * (Math.pow(Joint_Lower.length, 2.0) + Math.pow(Joint_Upper.cgRadius, 2.0))
            + Joint_Lower.moi
            + Joint_Upper.moi
            + 2
                * Joint_Upper.mass
                * Joint_Lower.length
                * Joint_Upper.cgRadius
                * Math.cos(position.get(1, 0)));
    M.set(
        1,
        0,
        Joint_Upper.mass * Math.pow(Joint_Upper.cgRadius, 2.0)
            + Joint_Upper.moi
            + Joint_Upper.mass * Joint_Lower.length * Joint_Upper.cgRadius * Math.cos(position.get(1, 0)));
    M.set(
        0,
        1,
        Joint_Upper.mass * Math.pow(Joint_Upper.cgRadius, 2.0)
            + Joint_Upper.moi
            + Joint_Upper.mass * Joint_Lower.length * Joint_Upper.cgRadius * Math.cos(position.get(1, 0)));
    M.set(1, 1, Joint_Upper.mass * Math.pow(Joint_Upper.cgRadius, 2.0) + Joint_Upper.moi);
    return M;
  }

  private Matrix<N2, N2> C(Vector<N2> position, Vector<N2> velocity) {
    var C = new Matrix<>(N2.instance, N2.instance);
    C.set(
        0,
        0,
        -Joint_Upper.mass
            * Joint_Lower.length
            * Joint_Upper.cgRadius
            * Math.sin(position.get(1, 0))
            * velocity.get(1, 0));
    C.set(
        1,
        0,
        Joint_Upper.mass
            * Joint_Lower.length
            * Joint_Upper.cgRadius
            * Math.sin(position.get(1, 0))
            * velocity.get(0, 0));
    C.set(
        0,
        1,
        -Joint_Upper.mass
            * Joint_Lower.length
            * Joint_Upper.cgRadius
            * Math.sin(position.get(1, 0))
            * (velocity.get(0, 0) + velocity.get(1, 0)));
    return C;
  }

  private Matrix<N2, N1> Tg(Vector<N2> position) {
    var Tg = new Matrix<>(N2.instance, N1.instance);
    Tg.set(
        0,
        0,
        (Joint_Lower.mass * Joint_Lower.cgRadius + Joint_Upper.mass * Joint_Lower.length)
                * g
                * Math.cos(position.get(0, 0))
            + Joint_Upper.mass
                * Joint_Upper.cgRadius
                * g
                * Math.cos(position.get(0, 0) + position.get(1, 0)));
    Tg.set(
        1,
        0,
        Joint_Upper.mass * Joint_Upper.cgRadius * g * Math.cos(position.get(0, 0) + position.get(1, 0)));
    return Tg;
  }

}