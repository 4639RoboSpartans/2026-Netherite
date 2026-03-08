/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.util;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team4639.frc2026.Constants;

/**
 * Holds a pair of constants that represent the same thing in Real and Simulated contexts.
 *
 * <p>Note: Instead of {@link RobotBase#isReal()} this relies on the Akit {@link
 * Constants#getMode()} value to determine the appropriate behavior. To facilitate deterministic log
 * replay, the sim constant will only emerge when the mode is {@link
 * org.team4639.frc2026.Constants.Mode#SIM} and the real constant will be used in {@link
 * org.team4639.frc2026.Constants.Mode#REAL} and {@link org.team4639.frc2026.Constants.Mode#REPLAY}
 * modes.
 */
public class SimRealConstantPair implements DoubleSupplier, Supplier<Double> {
  private final DoubleSupplier sim;
  private final DoubleSupplier real;
  private LoggedTunableNumber tunable = null;

  public SimRealConstantPair(DoubleSupplier sim, DoubleSupplier real) {
    this.sim = sim;
    this.real = real;
  }

  public SimRealConstantPair(double sim, DoubleSupplier real) {
    this(() -> sim, real);
  }

  public SimRealConstantPair(DoubleSupplier sim, double real) {
    this(sim, () -> real);
  }

  public SimRealConstantPair(double sim, double real) {
    this(() -> sim, () -> real);
  }

  /**
   * Constructs a pair that comes from two LoggedTunableNumbers. Only the relevant tunable is
   * constructed to not clog up the network.
   *
   * @param path the string path. On the real robot, this will be sent to NetworkTables exactly as
   *     is, while on the sim robot, it will have the suffix "Sim".
   */
  public static SimRealConstantPair ofTunableNumbers(String path) {
    DoubleSupplier realTunable, simTunable;
    LoggedTunableNumber tempTunable;
    if (Constants.currentMode == Constants.Mode.SIM) {
      tempTunable = new LoggedTunableNumber(path + "Sim");
      simTunable = tempTunable;
      realTunable = () -> 0;
    } else {
      tempTunable = new LoggedTunableNumber(path);
      realTunable = tempTunable;
      simTunable = () -> 0;
    }
    var pair = new SimRealConstantPair(simTunable, realTunable);
    pair.tunable = tempTunable;
    return pair;
  }

  /**
   * Constructs a pair that comes from two LoggedTunableNumbers. Only the relevant tunable is
   * constructed to not clog up the network.
   *
   * @param path the string path. On the real robot, this will be sent to NetworkTables exactly as
   *     is, while on the sim robot, it will have the suffix "Sim".
   */
  public static SimRealConstantPair ofTunableNumbers(
      String path, double simDefaultValue, double realDefaultValue) {
    DoubleSupplier realTunable, simTunable;
    LoggedTunableNumber tempTunable;
    if (Constants.currentMode == Constants.Mode.SIM) {
      tempTunable = new LoggedTunableNumber(path + "Sim").initDefault(simDefaultValue);
      simTunable = tempTunable;
      realTunable = () -> 0;
    } else {
      tempTunable = new LoggedTunableNumber(path).initDefault(realDefaultValue);
      realTunable = tempTunable;
      simTunable = () -> 0;
    }
    var pair = new SimRealConstantPair(simTunable, realTunable);
    pair.tunable = tempTunable;
    return pair;
  }

  /**
   * Constructs a pair that comes from two LoggedTunableNumbers. Only the relevant tunable is
   * constructed to not clog up the network.
   *
   * @param path the string path. On the real robot, this will be sent to NetworkTables exactly as
   *     is, while on the sim robot, it will have the suffix "Sim".
   */
  public static SimRealConstantPair ofTunableNumbers(String path, double bothDefaultValue) {
    return ofTunableNumbers(path, bothDefaultValue, bothDefaultValue);
  }

  @Override
  public double getAsDouble() {
    return Constants.currentMode == Constants.Mode.SIM ? sim.getAsDouble() : real.getAsDouble();
  }

  @Override
  public Double get() {
    return getAsDouble();
  }

  /**
   * If this pair is made from LoggedTunableNumbers, return the relevant number based on if it is
   * real or sim
   *
   * @return the LoggedTunableNumber object or {@link Optional#empty()} if it doesn't exist.
   */
  public Optional<LoggedTunableNumber> getLoggedTunableNumber() {
    return Optional.ofNullable(tunable);
  }
}
