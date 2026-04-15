/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * An instance of a controller that provides utils for applying deadzone to the controller axes and
 * double click detection
 */
public class DeadbandXboxController extends CommandXboxController {
    public static Set<DeadbandXboxController> instances = new HashSet<>();

    private final double DEFAULT_DEADZONE = 0.08;
    private final double DOUBLE_CLICK_TIME_WINDOW = 0.3; // 300ms window for double click

    private final Map<Trigger, Double> lastClickTime;
    private final Map<Trigger, Boolean> wasPressed;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public DeadbandXboxController(int port) {
        super(port);
        instances.add(this);

        // Initialize maps for double click detection
        lastClickTime = new HashMap<>();
        wasPressed = new HashMap<>();

        // Initialize all buttons
        initializeButton(a());
        initializeButton(b());
        initializeButton(x());
        initializeButton(y());
        initializeButton(leftBumper());
        initializeButton(rightBumper());
        initializeButton(back());
        initializeButton(start());
        initializeButton(leftStick());
        initializeButton(rightStick());
        initializeButton(leftTrigger());
        initializeButton(rightTrigger());
        initializeButton(povUp());
        initializeButton(povUpRight());
        initializeButton(povRight());
        initializeButton(povDownRight());
        initializeButton(povDown());
        initializeButton(povDownLeft());
        initializeButton(povLeft());
        initializeButton(povUpLeft());
        initializeButton(povCenter());
    }

    private void initializeButton(Trigger trigger) {
        lastClickTime.put(trigger, 0.0);
        wasPressed.put(trigger, false);
    }

    /**
     * Updates the double click detection state for a button. Should be called periodically (e.g., in
     * a periodic method).
     */
    public void updateDoubleClickDetection() {
        for (Trigger trigger : lastClickTime.keySet()) {
            boolean isPressed = trigger.getAsBoolean();
            boolean previouslyPressed = wasPressed.get(trigger);

            // Detect rising edge (button just pressed)
            if (isPressed && !previouslyPressed) {
                double currentTime = Timer.getFPGATimestamp();
                double timeSinceLastClick = currentTime - lastClickTime.get(trigger);

                // Update last click time
                lastClickTime.put(trigger, currentTime);
            }

            // Update previous state
            wasPressed.put(trigger, isPressed);
        }
    }

    /**
     * Creates a Trigger that activates on double click of the given trigger.
     *
     * @param trigger the trigger to detect double clicks on
     * @return a Trigger that is true when a double click is detected
     */
    public Trigger doubleClick(Trigger trigger) {
        return new Trigger(() -> {
            boolean isPressed = trigger.getAsBoolean();
            boolean previouslyPressed = wasPressed.getOrDefault(trigger, false);

            // Check for rising edge
            if (isPressed && !previouslyPressed) {
                double currentTime = Timer.getFPGATimestamp();
                double timeSinceLastClick = currentTime - lastClickTime.getOrDefault(trigger, 0.0);

                // Check if within double click window
                return timeSinceLastClick > 0 && timeSinceLastClick < DOUBLE_CLICK_TIME_WINDOW;
            }

            return false;
        });
    }

    // Convenience methods for double click detection on specific buttons
    public Trigger aDoubleClick() {
        return doubleClick(a());
    }

    public Trigger bDoubleClick() {
        return doubleClick(b());
    }

    public Trigger xDoubleClick() {
        return doubleClick(x());
    }

    public Trigger yDoubleClick() {
        return doubleClick(y());
    }

    public Trigger leftBumperDoubleClick() {
        return doubleClick(leftBumper());
    }

    public Trigger rightBumperDoubleClick() {
        return doubleClick(rightBumper());
    }

    public Trigger leftTriggerDoubleClick() {
        return doubleClick(leftTrigger());
    }

    public Trigger rightTriggerDoubleClick() {
        return doubleClick(rightTrigger());
    }

    public Trigger backDoubleClick() {
        return doubleClick(back());
    }

    public Trigger startDoubleClick() {
        return doubleClick(start());
    }

    public Trigger leftStickDoubleClick() {
        return doubleClick(leftStick());
    }

    public Trigger rightStickDoubleClick() {
        return doubleClick(rightStick());
    }

    /**
     * Get the underlying GenericHID object.
     *
     * @return the wrapped GenericHID object
     */
    @Override
    @Deprecated
    public XboxController getHID() {
        return super.getHID();
    }

    /**
     * Constructs an event instance around this button's digital signal.
     *
     * @param button the button index
     * @return an event instance representing the button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop()} default scheduler button loop.
     * @see #button(int, EventLoop)
     */
    @Override
    @Deprecated
    public Trigger button(int button) {
        return super.button(button);
    }

    /**
     * Constructs an event instance around this button's digital signal.
     *
     * @param button the button index
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the button's digital signal attached to the given loop.
     */
    @Override
    @Deprecated
    public Trigger button(int button, EventLoop loop) {
        return super.button(button, loop);
    }

    /**
     * Constructs a Trigger instance based around this angle of the default (index 0) POV on the HID,
     * attached to {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
     * loop}.
     *
     * <p>The POV angles start at 0 in the up direction, and increase clockwise (e.g. right is 90,
     * upper-left is 315).
     *
     * @param angle POV angle in degrees, or -1 for the center / not pressed.
     * @return a Trigger instance based around this angle of a POV on the HID.
     */
    @Override
    @Deprecated
    public Trigger pov(int angle) {
        return super.pov(angle);
    }

    /**
     * Constructs a Trigger instance based around this angle of a POV on the HID.
     *
     * <p>The POV angles start at 0 in the up direction, and increase clockwise (e.g. right is 90,
     * upper-left is 315).
     *
     * @param pov index of the POV to read (starting at 0). Defaults to 0.
     * @param angle POV angle in degrees, or -1 for the center / not pressed.
     * @param loop the event loop instance to attach the event to. Defaults to {@link
     *     CommandScheduler#getDefaultButtonLoop() the default command scheduler button loop}.
     * @return a Trigger instance based around this angle of a POV on the HID.
     */
    @Override
    @Deprecated
    public Trigger pov(int pov, int angle, EventLoop loop) {
        return super.pov(pov, angle, loop);
    }

    /**
     * Constructs a Trigger instance based around the 0 degree angle (up) of the default (index 0) POV
     * on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
     * scheduler button loop}.
     *
     * @return a Trigger instance based around the 0 degree angle of a POV on the HID.
     */
    @Override
    public Trigger povUp() {
        return super.povUp();
    }

    /**
     * Constructs a Trigger instance based around the 45 degree angle (right up) of the default (index
     * 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default
     * command scheduler button loop}.
     *
     * @return a Trigger instance based around the 45 degree angle of a POV on the HID.
     */
    @Override
    public Trigger povUpRight() {
        return super.povUpRight();
    }

    /**
     * Constructs a Trigger instance based around the 90 degree angle (right) of the default (index 0)
     * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
     * scheduler button loop}.
     *
     * @return a Trigger instance based around the 90 degree angle of a POV on the HID.
     */
    @Override
    public Trigger povRight() {
        return super.povRight();
    }

    /**
     * Constructs a Trigger instance based around the 135 degree angle (right down) of the default
     * (index 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the
     * default command scheduler button loop}.
     *
     * @return a Trigger instance based around the 135 degree angle of a POV on the HID.
     */
    @Override
    public Trigger povDownRight() {
        return super.povDownRight();
    }

    /**
     * Constructs a Trigger instance based around the 180 degree angle (down) of the default (index 0)
     * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
     * scheduler button loop}.
     *
     * @return a Trigger instance based around the 180 degree angle of a POV on the HID.
     */
    @Override
    public Trigger povDown() {
        return super.povDown();
    }

    /**
     * Constructs a Trigger instance based around the 225 degree angle (down left) of the default
     * (index 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the
     * default command scheduler button loop}.
     *
     * @return a Trigger instance based around the 225 degree angle of a POV on the HID.
     */
    @Override
    public Trigger povDownLeft() {
        return super.povDownLeft();
    }

    /**
     * Constructs a Trigger instance based around the 270 degree angle (left) of the default (index 0)
     * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
     * scheduler button loop}.
     *
     * @return a Trigger instance based around the 270 degree angle of a POV on the HID.
     */
    @Override
    public Trigger povLeft() {
        return super.povLeft();
    }

    /**
     * Constructs a Trigger instance based around the 315 degree angle (left up) of the default (index
     * 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default
     * command scheduler button loop}.
     *
     * @return a Trigger instance based around the 315 degree angle of a POV on the HID.
     */
    @Override
    public Trigger povUpLeft() {
        return super.povUpLeft();
    }

    /**
     * Constructs a Trigger instance based around the center (not pressed) position of the default
     * (index 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the
     * default command scheduler button loop}.
     *
     * @return a Trigger instance based around the center position of a POV on the HID.
     */
    @Override
    public Trigger povCenter() {
        return super.povCenter();
    }

    /**
     * Constructs a Trigger instance that is true when the axis value is less than {@code threshold},
     * attached to {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
     * loop}.
     *
     * @param axis The axis to read, starting at 0
     * @param threshold The value below which this trigger should return true.
     * @return a Trigger instance that is true when the axis value is less than the provided
     *     threshold.
     */
    @Override
    public Trigger axisLessThan(int axis, double threshold) {
        return super.axisLessThan(axis, threshold);
    }

    /**
     * Constructs a Trigger instance that is true when the axis value is less than {@code threshold},
     * attached to the given loop.
     *
     * @param axis The axis to read, starting at 0
     * @param threshold The value below which this trigger should return true.
     * @param loop the event loop instance to attach the trigger to
     * @return a Trigger instance that is true when the axis value is less than the provided
     *     threshold.
     */
    @Override
    public Trigger axisLessThan(int axis, double threshold, EventLoop loop) {
        return super.axisLessThan(axis, threshold, loop);
    }

    /**
     * Constructs a Trigger instance that is true when the axis value is less than {@code threshold},
     * attached to {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
     * loop}.
     *
     * @param axis The axis to read, starting at 0
     * @param threshold The value above which this trigger should return true.
     * @return a Trigger instance that is true when the axis value is greater than the provided
     *     threshold.
     */
    @Override
    public Trigger axisGreaterThan(int axis, double threshold) {
        return super.axisGreaterThan(axis, threshold);
    }

    /**
     * Constructs a Trigger instance that is true when the axis value is greater than {@code
     * threshold}, attached to the given loop.
     *
     * @param axis The axis to read, starting at 0
     * @param threshold The value above which this trigger should return true.
     * @param loop the event loop instance to attach the trigger to.
     * @return a Trigger instance that is true when the axis value is greater than the provided
     *     threshold.
     */
    @Override
    public Trigger axisGreaterThan(int axis, double threshold, EventLoop loop) {
        return super.axisGreaterThan(axis, threshold, loop);
    }

    /**
     * Constructs a Trigger instance that is true when the axis magnitude value is greater than {@code
     * threshold}, attached to the given loop.
     *
     * @param axis The axis to read, starting at 0
     * @param threshold The value above which this trigger should return true.
     * @param loop the event loop instance to attach the trigger to.
     * @return a Trigger instance that is true when the axis magnitude value is greater than the
     *     provided threshold.
     */
    @Override
    public Trigger axisMagnitudeGreaterThan(int axis, double threshold, EventLoop loop) {
        return super.axisMagnitudeGreaterThan(axis, threshold, loop);
    }

    /**
     * Constructs a Trigger instance that is true when the axis magnitude value is greater than {@code
     * threshold}, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
     * scheduler button loop}.
     *
     * @param axis The axis to read, starting at 0
     * @param threshold The value above which this trigger should return true.
     * @return a Trigger instance that is true when the deadbanded axis value is active (non-zero).
     */
    @Override
    public Trigger axisMagnitudeGreaterThan(int axis, double threshold) {
        return super.axisMagnitudeGreaterThan(axis, threshold);
    }

    /**
     * Get the value of the axis.
     *
     * @param axis The axis to read, starting at 0.
     * @return The value of the axis.
     */
    @Override
    public double getRawAxis(int axis) {
        return super.getRawAxis(axis);
    }

    /**
     * Set the rumble output for the HID. The DS currently supports 2 rumble values, left rumble and
     * right rumble.
     *
     * @param type Which rumble value to set
     * @param value The normalized value (0 to 1) to set the rumble to
     */
    @Override
    public void setRumble(GenericHID.RumbleType type, double value) {
        super.setRumble(type, value);
    }

    /**
     * Get if the HID is connected.
     *
     * @return true if the HID is connected
     */
    @Override
    public boolean isConnected() {
        return super.isConnected();
    }

    /**
     * Constructs a Trigger instance around the A button's digital signal.
     *
     * @return a Trigger instance representing the A button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #a(EventLoop)
     */
    @Override
    public Trigger a() {
        return super.a();
    }

    /**
     * Constructs a Trigger instance around the A button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the A button's digital signal attached to the given
     *     loop.
     */
    @Override
    public Trigger a(EventLoop loop) {
        return super.a(loop);
    }

    /**
     * Constructs a Trigger instance around the B button's digital signal.
     *
     * @return a Trigger instance representing the B button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #b(EventLoop)
     */
    @Override
    public Trigger b() {
        return super.b();
    }

    /**
     * Constructs a Trigger instance around the B button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the B button's digital signal attached to the given
     *     loop.
     */
    @Override
    public Trigger b(EventLoop loop) {
        return super.b(loop);
    }

    /**
     * Constructs a Trigger instance around the X button's digital signal.
     *
     * @return a Trigger instance representing the X button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #x(EventLoop)
     */
    @Override
    public Trigger x() {
        return super.x();
    }

    /**
     * Constructs a Trigger instance around the X button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the X button's digital signal attached to the given
     *     loop.
     */
    @Override
    public Trigger x(EventLoop loop) {
        return super.x(loop);
    }

    /**
     * Constructs a Trigger instance around the Y button's digital signal.
     *
     * @return a Trigger instance representing the Y button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #y(EventLoop)
     */
    @Override
    public Trigger y() {
        return super.y();
    }

    /**
     * Constructs a Trigger instance around the Y button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the Y button's digital signal attached to the given
     *     loop.
     */
    @Override
    public Trigger y(EventLoop loop) {
        return super.y(loop);
    }

    /**
     * Constructs a Trigger instance around the left bumper button's digital signal.
     *
     * @return a Trigger instance representing the left bumper button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #leftBumper(EventLoop)
     */
    @Override
    public Trigger leftBumper() {
        return super.leftBumper();
    }

    /**
     * Constructs a Trigger instance around the left bumper button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the left bumper button's digital signal attached to the
     *     given loop.
     */
    @Override
    public Trigger leftBumper(EventLoop loop) {
        return super.leftBumper(loop);
    }

    /**
     * Constructs a Trigger instance around the right bumper button's digital signal.
     *
     * @return a Trigger instance representing the right bumper button's digital signal attached to
     *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #rightBumper(EventLoop)
     */
    @Override
    public Trigger rightBumper() {
        return super.rightBumper();
    }

    /**
     * Constructs a Trigger instance around the right bumper button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the right bumper button's digital signal attached to
     *     the given loop.
     */
    @Override
    public Trigger rightBumper(EventLoop loop) {
        return super.rightBumper(loop);
    }

    /**
     * Constructs a Trigger instance around the back button's digital signal.
     *
     * @return a Trigger instance representing the back button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #back(EventLoop)
     */
    @Override
    public Trigger back() {
        return super.back();
    }

    /**
     * Constructs a Trigger instance around the back button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the back button's digital signal attached to the given
     *     loop.
     */
    @Override
    public Trigger back(EventLoop loop) {
        return super.back(loop);
    }

    /**
     * Constructs a Trigger instance around the start button's digital signal.
     *
     * @return a Trigger instance representing the start button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #start(EventLoop)
     */
    @Override
    public Trigger start() {
        return super.start();
    }

    /**
     * Constructs a Trigger instance around the start button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the start button's digital signal attached to the given
     *     loop.
     */
    @Override
    public Trigger start(EventLoop loop) {
        return super.start(loop);
    }

    /**
     * Constructs a Trigger instance around the left stick button's digital signal.
     *
     * @return a Trigger instance representing the left stick button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #leftStick(EventLoop)
     */
    @Override
    public Trigger leftStick() {
        return super.leftStick();
    }

    /**
     * Constructs a Trigger instance around the left stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the left stick button's digital signal attached to the
     *     given loop.
     */
    @Override
    public Trigger leftStick(EventLoop loop) {
        return super.leftStick(loop);
    }

    /**
     * Constructs a Trigger instance around the right stick button's digital signal.
     *
     * @return a Trigger instance representing the right stick button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #rightStick(EventLoop)
     */
    @Override
    public Trigger rightStick() {
        return super.rightStick();
    }

    /**
     * Constructs a Trigger instance around the right stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the right stick button's digital signal attached to the
     *     given loop.
     */
    @Override
    public Trigger rightStick(EventLoop loop) {
        return super.rightStick(loop);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
     *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @param loop the event loop instance to attach the Trigger to.
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
     *     threshold, attached to the given event loop
     */
    @Override
    public Trigger leftTrigger(double threshold, EventLoop loop) {
        return super.leftTrigger(threshold, loop);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
     *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
     *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *     button loop}.
     */
    @Override
    public Trigger leftTrigger(double threshold) {
        return super.leftTrigger(threshold);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the left trigger's axis exceeds 0.5, attached to
     *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    @Override
    public Trigger leftTrigger() {
        return super.leftTrigger();
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
     *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @param loop the event loop instance to attach the Trigger to.
     * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
     *     threshold, attached to the given event loop
     */
    @Override
    public Trigger rightTrigger(double threshold, EventLoop loop) {
        return super.rightTrigger(threshold, loop);
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
     *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
     *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *     button loop}.
     */
    @Override
    public Trigger rightTrigger(double threshold) {
        return super.rightTrigger(threshold);
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the right trigger's axis exceeds 0.5, attached to
     *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    @Override
    public Trigger rightTrigger() {
        return super.rightTrigger();
    }

    /**
     * Get the X axis value of left side of the controller. Right is positive.
     *
     * @return The axis value.
     */
    @Override
    public double getLeftX() {
        return applyDeadzone(super.getLeftX());
    }

    public double getLeftXInverted() {
        return -getLeftX();
    }

    /**
     * Get the X axis value of right side of the controller. Right is positive.
     *
     * @return The axis value.
     */
    @Override
    public double getRightX() {
        return applyDeadzone(super.getRightX());
    }

    public double getRightXInverted() {
        return -getRightX();
    }

    /**
     * Get the Y axis value of left side of the controller. Back is positive.
     *
     * @return The axis value.
     */
    @Override
    public double getLeftY() {
        return applyDeadzone(super.getLeftY());
    }

    public double getLeftYInverted() {
        return -getLeftY();
    }

    /**
     * Get the Y axis value of right side of the controller. Back is positive.
     *
     * @return The axis value.
     */
    @Override
    public double getRightY() {
        return applyDeadzone(super.getRightY());
    }

    public double getRightYInverted() {
        return -getRightY();
    }

    /**
     * Get the left trigger axis value of the controller. Note that this axis is bound to the range of
     * [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    @Override
    public double getLeftTriggerAxis() {
        return super.getLeftTriggerAxis();
    }

    /**
     * Get the right trigger axis value of the controller. Note that this axis is bound to the range
     * of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    @Override
    public double getRightTriggerAxis() {
        return super.getRightTriggerAxis();
    }

    public double applyDeadzone(double value) {
        return Math.signum(value) * Math.max((Math.abs(value) - DEFAULT_DEADZONE), 0);
    }
}
