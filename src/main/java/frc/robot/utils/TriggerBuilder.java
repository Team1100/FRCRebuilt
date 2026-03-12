package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Tool for easily creating full maps of `Triggers` to `Commands`, primarily
 * created to easily support submaps.
 * Only slightly overengineered.
 */
public class TriggerBuilder<S> {
	private S m_targetSubmap;
	private Referrable<S> m_submapHolder;

	private ArrayList<Binding> m_bindings;

	@FunctionalInterface
	public interface CommandConsumer {
		void accept(Trigger trigger, Command command);
	}

	@FunctionalInterface
	public interface SwitchIndicator {
		void accept(boolean switching);
	}

	private static class Binding {
		public Trigger trigger;
		private final Command command;
		private final CommandConsumer commandConsumer;

		public Binding(Trigger t, Command c, CommandConsumer cc) {
			trigger = t;
			command = c;
			commandConsumer = cc;
		}

		public void bind() {
			commandConsumer.accept(trigger, command);
		}
	}

	public TriggerBuilder(Referrable<S> currentSubmap) {
		m_targetSubmap = null;
		m_submapHolder = currentSubmap;
		m_bindings = new ArrayList<>();
	}

	public TriggerBuilder<S> map(Trigger trigger, Command command, CommandConsumer commandConsumer) {
		if (m_targetSubmap != null) {
			S s = m_targetSubmap;
			trigger = trigger.and(()->{
				return m_submapHolder.get() == s;
			});
		}
		m_bindings.add(new Binding(trigger, command, commandConsumer));
		return this;
	}

	public TriggerBuilder<S> whileTrue(Trigger trigger, Command command) {
		map(trigger, command, Trigger::whileTrue);
		return this;
	}

	public TriggerBuilder<S> onTrue(Trigger trigger, Command command) {
		map(trigger, command, Trigger::onTrue);
		return this;
	}

	public TriggerBuilder<S> beginSubmap(S submap) {
		assert m_targetSubmap == null;
		m_targetSubmap = submap;
		return this;
	}

	public TriggerBuilder<S> endSubmap() {
		assert m_targetSubmap != null;
		m_targetSubmap = null;
		return this;
	}

	private static class SwitchSubmapCommand<S> extends Command {
		private final double m_time;
		private final Trigger m_trigger;
		private final SwitchIndicator m_switchIndicator;
		private final Referrable<S> m_submapHolder;
		private final S m_newSubmap;

		public SwitchSubmapCommand(Trigger trigger, SwitchIndicator switchIndicator, Referrable<S> submapHolder, S newSubmap) {
			m_time = Timer.getFPGATimestamp();
			m_trigger = trigger;
			m_switchIndicator = switchIndicator;
			m_submapHolder = submapHolder;
			m_newSubmap = newSubmap;
		}

		@Override
		public void initialize() {
			m_switchIndicator.accept(true);
		}

		@Override
		public void execute() {}

		@Override
		public void end(boolean interrupted) {
			m_submapHolder.set(m_newSubmap);
			m_switchIndicator.accept(false);
		}

		@Override
		public boolean isFinished() {
			return (Timer.getFPGATimestamp() > m_time + 0.5) && !m_trigger.getAsBoolean();
		}
	}

	public TriggerBuilder<S> switchSubmap(SwitchIndicator switchIndicator, Trigger trigger, S newSubmap) {
		onTrue(trigger, new SwitchSubmapCommand<S>(trigger, switchIndicator, m_submapHolder, newSubmap));
		return this;
	}

	public TriggerBuilder<S> debounceLast(double debounce) {
		Binding last = m_bindings.get(m_bindings.size()-1);
		last.trigger = last.trigger.debounce(debounce);
		return this;
	}

	public void register() {
		for (Binding binding : m_bindings) binding.bind();
	}

	public static class RumbleIndicator implements SwitchIndicator {
		private GenericHID m_controller;

		public RumbleIndicator(GenericHID controller) {
			m_controller = controller;
		}

		public void accept(boolean switching) {
			m_controller.setRumble(RumbleType.kBothRumble, switching ? 1 : 0);
		}
	}

	public static class PrintIndicator implements SwitchIndicator {
		public void accept(boolean switching) {
			System.out.println("Switching: " + switching);
		}
	}
}
