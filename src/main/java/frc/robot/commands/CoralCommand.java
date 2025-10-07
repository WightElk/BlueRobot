package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.CoralMechanism;

public class CoralCommand extends Command {
    private final CoralMechanism coralMech;
    private final DoubleSupplier speed;
    private boolean into = true;

    public CoralCommand(CoralMechanism coral, DoubleSupplier speed, boolean into)
    {
        System.out.println("CoralIn CONSTRUCTOR ");
        coralMech = coral;
        this.speed = speed;
        this.into = into;
        addRequirements(coral);
    }

    @Override
    public void initialize() {
        coralMech.reset();
    }
  
    @Override
    public void execute() {
        coralMech.setSpeed(speed.getAsDouble(), into);
    }
  
    @Override
    public void end(boolean interrupted) {
        coralMech.setSpeed1(0);
    }
  
    @Override
    public boolean isFinished() {
        //System.out.println("CoralIn isFinished ");
        return coralMech.isCoralIn();
    }
}
