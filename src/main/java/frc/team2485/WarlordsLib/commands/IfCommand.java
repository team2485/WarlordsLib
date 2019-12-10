package frc.team2485.WarlordsLib.commands;

import edu.wpi.first.wpilibj.frc2.command.Command;
import edu.wpi.first.wpilibj.frc2.command.ConditionalCommand;
import edu.wpi.first.wpilibj.frc2.command.SendableCommandBase;

import java.util.function.BooleanSupplier;

import static java.util.Objects.requireNonNull;

/**
 * Runs one of two commands, depending on the value of the given condition when this command is
 * initialized.  Does not actually schedule the selected command - rather, the command is run
 * through this command; this ensures that the command will behave as expected if used as part of a
 * CommandGroup.  Requires the requirements of both commands, again to ensure proper functioning
 * when used in a CommandGroup.  If this is undesired, consider using {@link edu.wpi.first.wpilibj.frc2.command.ScheduleCommand}.
 *
 * <p>As this command contains multiple component commands within it, it is technically a command
 * group; the command instances that are passed to it cannot be added to any other groups, or
 * scheduled individually.
 *
 * <p>As a rule, CommandGroups require the union of the requirements of their component commands.
 */
public class IfCommand extends SendableCommandBase {

    private final Command m_onTrue;
    private final BooleanSupplier m_condition;
    private boolean conditionIsTrue;
    /**
     * Creates a new ConditionalCommand.
     *
     * @param onTrue    the command to run if the condition is true
     * @param condition the condition to determine which command to run
     */
    public IfCommand(Command onTrue, BooleanSupplier condition) {
        m_onTrue = onTrue;
        m_condition = requireNonNull(condition);
        m_requirements.addAll(m_onTrue.getRequirements());
        conditionIsTrue = false;
    }

    @Override
    public void initialize() {
        if (m_condition.getAsBoolean()) {
            conditionIsTrue = true;
            m_onTrue.initialize();
        }
    }

    @Override
    public void execute() {
        if(conditionIsTrue) {
            m_onTrue.execute();
        }

    }

    @Override
    public void end(boolean interrupted) {
        if(conditionIsTrue) {
            m_onTrue.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        if(conditionIsTrue) {
            return m_onTrue.isFinished();
        } else {
            return true;
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_onTrue.runsWhenDisabled();
    }
}

