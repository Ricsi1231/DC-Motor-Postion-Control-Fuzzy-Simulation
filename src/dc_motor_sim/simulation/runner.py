"""The closed-loop simulation driver.

One loop serves every controller. Each control step:

1. the encoder samples the true shaft angle (quantization + noise),
2. the error and its derivative are computed against the setpoint,
3. the controller produces a control signal, scaled into a voltage,
4. the motor integrates that voltage over ``substeps`` sub-intervals,
5. convergence is tested against the error and its derivative.
"""

from __future__ import annotations

import logging

import numpy as np

from dc_motor_sim.config import (
    DEFAULT_ENCODER,
    DEFAULT_MOTOR,
    DEFAULT_SIM,
    EncoderParams,
    MotorParams,
    SimParams,
)
from dc_motor_sim.control.base import PositionController
from dc_motor_sim.model.dc_motor import DCMotorModel
from dc_motor_sim.sensors.encoder import RotaryEncoder
from dc_motor_sim.simulation.result import SimulationResult

__all__ = ["run_simulation"]

logger = logging.getLogger(__name__)


def run_simulation(
    controller: PositionController,
    start_deg: float,
    target_deg: float,
    *,
    sim_params: SimParams = DEFAULT_SIM,
    motor_params: MotorParams = DEFAULT_MOTOR,
    encoder_params: EncoderParams = DEFAULT_ENCODER,
    rng: np.random.Generator | None = None,
) -> SimulationResult:
    """Run a closed-loop position-control simulation.

    Args:
        controller: Any :class:`~dc_motor_sim.control.base.PositionController`.
            It is reset and re-targeted before the run begins.
        start_deg: Initial shaft position (degrees).
        target_deg: Desired shaft position (degrees).
        sim_params: Loop timing, integration, and convergence settings.
        motor_params: Motor physical constants.
        encoder_params: Encoder resolution and noise settings.
        rng: Random generator for encoder noise. Pass a seeded generator to
            make the run reproducible.

    Returns:
        A :class:`~dc_motor_sim.simulation.result.SimulationResult` holding the
        full trace.
    """
    motor = DCMotorModel(initial_position_deg=start_deg, params=motor_params)
    encoder = RotaryEncoder(params=encoder_params, rng=rng)

    controller.reset()
    controller.set_target(target_deg)

    dt = sim_params.dt
    substep_dt = sim_params.substep_dt

    actual_position = start_deg
    measured_position = encoder.read_position(actual_position)
    previous_error = target_deg - measured_position

    time = [0.0]
    actual_positions = [actual_position]
    measured_positions = [measured_position]
    targets = [target_deg]
    errors = [previous_error]
    controls = [0.0]
    voltages = [0.0]
    currents = [0.0]
    velocities = [0.0]

    logger.info(
        "Starting closed-loop simulation | controller=%s | encoder=%d PPR (%.3f deg/count), "
        "noise=%.3f deg | start=%.2f deg | target=%.2f deg | dt=%.2f ms",
        controller.name,
        encoder.ppr,
        encoder.get_resolution(),
        encoder.noise_std,
        start_deg,
        target_deg,
        dt * 1000,
    )

    step = 0
    converged = False

    while step < sim_params.max_steps:
        error = target_deg - measured_position
        delta_error = error - previous_error

        control_signal = controller.compute(measured_position, dt)
        voltage = control_signal * sim_params.voltage_scale

        for _ in range(sim_params.substeps):
            motor.step(voltage, substep_dt)

        actual_position = motor.get_position_deg()
        velocity = motor.get_velocity_deg_per_sec()
        current = motor.get_current()
        measured_position = encoder.read_position(actual_position)

        step += 1
        time.append(step * dt)
        actual_positions.append(actual_position)
        measured_positions.append(measured_position)
        targets.append(target_deg)
        errors.append(error)
        controls.append(control_signal)
        voltages.append(voltage)
        currents.append(current)
        velocities.append(velocity)
        previous_error = error

        if step % sim_params.display_interval == 0:
            logger.info(
                "Step %d (%.3f s): actual=%.2f deg, measured=%.2f deg, error=%.2f deg, "
                "V=%.2f, count=%d",
                step,
                step * dt,
                actual_position,
                measured_position,
                error,
                voltage,
                encoder.get_count(),
            )

        if (
            abs(error) < sim_params.convergence_position
            and abs(delta_error) < sim_params.convergence_delta
        ):
            converged = True
            logger.info(
                "Converged at step %d (%.3f s): actual=%.2f deg, measured=%.2f deg, "
                "error=%.2f deg, velocity=%.2f deg/s, current=%.4f A, count=%d",
                step,
                step * dt,
                actual_position,
                measured_position,
                error,
                velocity,
                current,
                encoder.get_count(),
            )
            break

    if not converged:
        logger.warning(
            "Reached maximum steps (%d) without converging: actual=%.2f deg, "
            "measured=%.2f deg, error=%.2f deg",
            sim_params.max_steps,
            actual_position,
            measured_position,
            errors[-1],
        )

    return SimulationResult(
        time=np.asarray(time, dtype=float),
        actual_position=np.asarray(actual_positions, dtype=float),
        measured_position=np.asarray(measured_positions, dtype=float),
        target=np.asarray(targets, dtype=float),
        error=np.asarray(errors, dtype=float),
        control=np.asarray(controls, dtype=float),
        voltage=np.asarray(voltages, dtype=float),
        current=np.asarray(currents, dtype=float),
        velocity=np.asarray(velocities, dtype=float),
        steps=step,
        converged=converged,
        controller_name=controller.name,
        start_deg=start_deg,
        target_deg=target_deg,
    )
