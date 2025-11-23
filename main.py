import sys
import numpy as np
from fuzzy_controller import FuzzyMotorController
from dc_motor_model import DCMotorModel
from encoder_sensor import RotaryEncoder
from visualization import (plot_membership_functions, plot_simulation_results,
                          plot_control_surface, plot_final_summary)
from motor_parameters import (MOTOR_RESPONSE_GAIN, MAX_SIMULATION_STEPS,
                              CONVERGENCE_THRESHOLD_POSITION, CONVERGENCE_THRESHOLD_DELTA,
                              DELTA_ERROR_MIN, DELTA_ERROR_MAX,
                              POSITION_MIN, POSITION_MAX, DISPLAY_INTERVAL,
                              VOLTAGE_SCALE, DT, ENCODER_PPR, ENCODER_NOISE_STD)


def simulate_motor_control(current_position, target_position, controller):
    """
    Simulate closed-loop motor control with encoder feedback.

    Args:
        current_position: Initial motor position in degrees
        target_position: Desired position in degrees
        controller: FuzzyMotorController instance

    Returns:
        Tuple of (time_steps, actual_positions, measured_positions, targets, errors, control_signals, steps)
    """
    motor = DCMotorModel(initial_position_deg=current_position)
    encoder = RotaryEncoder(pulses_per_revolution=ENCODER_PPR, noise_std=ENCODER_NOISE_STD)

    actual_position = current_position
    measured_position = encoder.read_position(actual_position)
    previous_error = target_position - measured_position

    actual_positions = [actual_position]
    measured_positions = [measured_position]
    targets = [target_position]
    errors = [previous_error]
    control_signals = [0.0]
    voltages = [0.0]
    currents = [0.0]
    velocities = [0.0]
    time_steps = [0.0]

    max_steps = MAX_SIMULATION_STEPS
    convergence_threshold = CONVERGENCE_THRESHOLD_POSITION
    step = 0

    print(f"\nStarting closed-loop simulation with encoder feedback:")
    print(f"Encoder resolution: {ENCODER_PPR} PPR ({encoder.get_resolution():.3f}°/count)")
    print(f"Encoder noise: {ENCODER_NOISE_STD}° std dev")
    print(f"Initial position: {current_position}°")
    print(f"Target position: {target_position}°")
    print(f"Initial error: {previous_error}°")
    print(f"Time step: {DT*1000:.2f} ms\n")

    while step < max_steps:
        error = target_position - measured_position
        delta_error = error - previous_error

        delta_error = np.clip(delta_error, DELTA_ERROR_MIN, DELTA_ERROR_MAX)

        control_signal = controller.compute_control(error, delta_error)

        voltage = control_signal * VOLTAGE_SCALE

        substeps = 10
        substep_dt = DT / substeps
        for _ in range(substeps):
            motor.step(voltage, dt=substep_dt)

        actual_position = motor.get_position_deg()
        velocity = motor.get_velocity_deg_per_sec()
        current = motor.get_current()

        measured_position = encoder.read_position(actual_position)

        step += 1
        time_steps.append(step * DT)
        actual_positions.append(actual_position)
        measured_positions.append(measured_position)
        targets.append(target_position)
        errors.append(error)
        control_signals.append(control_signal)
        voltages.append(voltage)
        currents.append(current)
        velocities.append(velocity)
        previous_error = error

        if step % DISPLAY_INTERVAL == 0:
            print(f"Step {step} ({step*DT:.3f}s): Actual={actual_position:.2f}°, "
                  f"Measured={measured_position:.2f}°, Err={error:.2f}°, "
                  f"V={voltage:.2f}V, Count={encoder.get_count()}")

        if abs(error) < convergence_threshold and abs(delta_error) < CONVERGENCE_THRESHOLD_DELTA:
            print(f"\nConverged at step {step} ({step*DT:.3f}s)!")
            print(f"Final actual position: {actual_position:.2f}°")
            print(f"Final measured position: {measured_position:.2f}°")
            print(f"Final error: {error:.2f}°")
            print(f"Final velocity: {velocity:.2f}°/s")
            print(f"Final current: {current:.4f} A")
            print(f"Final encoder count: {encoder.get_count()}")
            break

    if step >= max_steps:
        print(f"\nReached maximum steps ({max_steps})")
        print(f"Final actual position: {actual_position:.2f}°")
        print(f"Final measured position: {measured_position:.2f}°")
        print(f"Final error: {errors[-1]:.2f}°")

    return time_steps, actual_positions, measured_positions, targets, errors, control_signals, step


def main():
    """
    Main entry point for DC motor fuzzy position controller simulation.
    """
    if len(sys.argv) != 3:
        print("Usage: python main.py <current_position> <target_position>")
        print("Example: python main.py 0 90")
        sys.exit(1)

    try:
        current_position = float(sys.argv[1])
        target_position = float(sys.argv[2])
    except ValueError:
        print("Error: Positions must be numeric values")
        sys.exit(1)

    if not (POSITION_MIN <= current_position <= POSITION_MAX and POSITION_MIN <= target_position <= POSITION_MAX):
        print(f"Error: Positions must be between {POSITION_MIN} and {POSITION_MAX} degrees")
        sys.exit(1)

    print("=" * 60)
    print("DC Motor Fuzzy Position Controller")
    print("with Realistic Motor Physics Model")
    print("=" * 60)

    controller = FuzzyMotorController()

    print("\nDisplaying membership functions...")
    plot_membership_functions(controller)

    print("\nDisplaying control surface...")
    plot_control_surface(controller)

    time_steps, actual_positions, measured_positions, targets, errors, control_signals, steps = simulate_motor_control(
        current_position, target_position, controller
    )

    print("\nDisplaying simulation results...")
    plot_simulation_results(time_steps, actual_positions, targets, errors, control_signals, measured_positions)

    print("\nDisplaying final summary...")
    plot_final_summary(current_position, target_position, actual_positions[-1], steps)

    print("\n" + "=" * 60)
    print("Simulation completed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main()
