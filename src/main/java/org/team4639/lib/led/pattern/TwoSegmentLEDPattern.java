/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern;

import edu.wpi.first.wpilibj.util.Color;

public class TwoSegmentLEDPattern implements LEDPattern {
    private final LEDPattern pattern1;
    private final LEDPattern pattern2;

    private final int n1Inclusive;
    private final int n2Exclusive;

    private final int n3Inclusive;
    private final int n4Exclusive;

    public TwoSegmentLEDPattern(
            LEDPattern pattern1,
            LEDPattern pattern2,
            int n1Inclusive,
            int n2Exclusive,
            int n3Inclusive,
            int n4Exclusive,
            boolean invertSecondSegment) {
        this.pattern1 = pattern1;
        this.pattern2 = pattern2;
        this.n1Inclusive = n1Inclusive;
        this.n2Exclusive = n2Exclusive;
        this.n3Inclusive = n3Inclusive;
        this.n4Exclusive = n4Exclusive;
        this.invertSecondSegment = invertSecondSegment;
        assert n1Inclusive < n2Exclusive && n3Inclusive < n4Exclusive && n3Inclusive >= n2Exclusive;
    }

    private final boolean invertSecondSegment;

    @Override
    public Color get(int led, double time) {
        if (led >= n1Inclusive && led < n2Exclusive) return pattern1.get(led, time);
        else if (led >= n3Inclusive && led < n4Exclusive)
            return pattern2.get(invertSecondSegment ? (n4Exclusive - led + 1) : (led - n3Inclusive), time);
        else return Color.kBlack;
    }
}
