/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.team4639.lib.led.pattern.LEDPattern;

public class RGBPattern implements LEDPattern {
  private final LEDPattern pattern;
  private final Color8Bit red;
  private final Color8Bit green;
  private final Color8Bit blue;

  public RGBPattern(LEDPattern pattern, Color8Bit red, Color8Bit green, Color8Bit blue) {
    this.pattern = pattern;
    this.red = red;
    this.green = green;
    this.blue = blue;
  }

  @Override
  public Color get(int led, double time) {
    Color8Bit color = new Color8Bit(pattern.get(led, time));
    Color8Bit transformedColor =
        new Color8Bit(
            collapse(color, this.red), collapse(color, this.green), collapse(color, this.blue));

    return new Color(transformedColor);
  }

  private int[] toIntArray(Color8Bit color) {
    return new int[] {color.red, color.green, color.blue};
  }

  private int collapse(Color8Bit color, Color8Bit mask) {
    int[] colorArray = toIntArray(color);
    int[] maskArray = toIntArray(mask);

    double sum = 0;
    for (int i = 0; i < colorArray.length; i++) {
      sum += (int) (colorArray[i] * maskArray[i]) / (255.0);
    }

    return (int) MathUtil.clamp(sum, 0, 255);
  }

  public static LEDPattern GRB(LEDPattern pattern) {
    return new RGBPattern(
        pattern, new Color8Bit(0, 255, 0), new Color8Bit(255, 0, 0), new Color8Bit(0, 0, 255));
  }

  public static LEDPattern GRB(Color color) {
    return new RGBPattern(
        LEDPattern.ofColor(color),
        new Color8Bit(0, 255, 0),
        new Color8Bit(255, 0, 0),
        new Color8Bit(0, 0, 255));
  }
}
