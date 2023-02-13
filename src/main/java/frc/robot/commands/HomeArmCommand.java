package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class HomeArmCommand extends ParallelCommandGroup {

    public HomeArmCommand() {

        // TODO - 0.0 is most likely not the right values, we
        // need to determine what the minimum value is for the
        // arm's height and reach when it is in its home
        // position.
        addCommands(new SetElevatorHeightCommand(0.0),
                new SetArmReachCommand(0.0));
    }
}
