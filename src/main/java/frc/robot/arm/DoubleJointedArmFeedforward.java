package frc.robot.arm;

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Calculates feedforward voltages for a double jointed arm.
 *
 * <p>https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 */
class DoubleJointedArmFeedforward {
    private static final double G = 9.80665;

    public static class JointConfig {
        double _mass;
        double _length; 
        double _moi; 
        double _cgRadius; 
        DCMotor _motor;

        JointConfig(double mass, double length, double moi, double cgRadius, DCMotor motor){
            this._mass = mass;
            this._length = length;
            this._moi = moi;
            this._cgRadius = cgRadius;
            this._motor = motor;
        }

        public double getMass(){return this._mass;}
        public double getLength(){return this._length;}
        public double getMOI(){return this._moi;}
        public double getCGRadius(){return this._cgRadius;}
        public DCMotor getMotor(){return this._motor;}
    }

  private final JointConfig joint_1;
  private final JointConfig joint_2;

  public DoubleJointedArmFeedforward(JointConfig joint_1, JointConfig joint_2) {
    this.joint_1 = joint_1;
    this.joint_2 = joint_2;
  }

  public Vector<N2> calculate(Vector<N2> position) {
    return calculate(position, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0, 0.0));
  }

  public Vector<N2> calculate(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
    var M = new Matrix<>(N2.instance, N2.instance);
    var C = new Matrix<>(N2.instance, N2.instance);
    var Tg = new Matrix<>(N2.instance, N1.instance);

    M.set(
        0,
        0,
        joint_1.getMass() * Math.pow(joint_1.getCGRadius(), 2.0)
            + joint_2.getMass() * (Math.pow(joint_1.getLength(), 2.0) + Math.pow(joint_2.getCGRadius(), 2.0))
            + joint_1.getMOI()
            + joint_2.getMOI()
            + 2
                * joint_2.getMass()
                * joint_1.getLength()
                * joint_2.getCGRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        1,
        0,
        joint_2.getMass() * Math.pow(joint_2.getCGRadius(), 2.0)
            + joint_2.getMOI()
            + joint_2.getMOI()
                * joint_1.getLength()
                * joint_2.getCGRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        0,
        1,
        joint_2.getMass() * Math.pow(joint_2.getCGRadius(), 2.0)
            + joint_2.getMOI()
            + joint_2.getMOI()
                * joint_1.getLength()
                * joint_2.getCGRadius()
                * Math.cos(position.get(1, 0)));
    M.set(1, 1, joint_2.getMass() * Math.pow(joint_2.getCGRadius(), 2.0) + joint_2.getMOI());
    C.set(
        0,
        0,
        -joint_2.getMass()
            * joint_1.getLength()
            * joint_2.getCGRadius()
            * Math.sin(position.get(1, 0))
            * velocity.get(1, 0));
    C.set(
        1,
        0,
        joint_2.getMass()
            * joint_1.getLength()
            * joint_2.getCGRadius()
            * Math.sin(position.get(1, 0))
            * velocity.get(0, 0));
    C.set(
        0,
        1,
        -joint_2.getMass()
            * joint_1.getLength()
            * joint_2.getCGRadius()
            * Math.sin(position.get(1, 0))
            * (velocity.get(0, 0) + velocity.get(1, 0)));
    Tg.set(
        0,
        0,
        (joint_1.getMass() * joint_1.getCGRadius() + joint_2.getMass() * joint_1.getLength())
                * G
                * Math.cos(position.get(0, 0))
            + joint_2.getMass()
                * joint_2.getCGRadius()
                * G
                * Math.cos(position.get(0, 0) + position.get(1, 0)));
    Tg.set(
        1,
        0,
        joint_2.getMass()
            * joint_2.getCGRadius()
            * G
            * Math.cos(position.get(0, 0) + position.get(1, 0)));

    var torque = M.times(acceleration).plus(C.times(velocity)).plus(Tg);
    return VecBuilder.fill(
        joint_1.getMotor().getVoltage(torque.get(0, 0), velocity.get(0, 0)),
        joint_2.getMotor().getVoltage(torque.get(1, 0), velocity.get(1, 0)));
  }
}