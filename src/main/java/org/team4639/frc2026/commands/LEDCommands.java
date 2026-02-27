package org.team4639.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.subsystems.ledkicker.LEDKicker;
import org.team4639.lib.led.pattern.LEDPattern;

public class LEDCommands {
    private static final class defaultSchema extends Command {
        private LEDPattern pattern = null;
        private final LEDKicker leds;
        private final RobotState state;

        public defaultSchema(LEDKicker leds, RobotState state){
            this.leds = leds;
            this.state = state;

            addRequirements(leds);
        }

        @Override
        public void execute() {
            LEDPattern newPattern = state.getDesiredLEDPattern();
            if (pattern != newPattern){
                leds.setPattern(newPattern);
            }
            pattern = newPattern;
        }
    }

    public static Command useDefaultSchema(LEDKicker leds, RobotState state){
        return new defaultSchema(leds, state);
    }
}
