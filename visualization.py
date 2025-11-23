import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plot_membership_functions(controller):
    """
    Plot membership functions for error, delta error, and control signal.

    Args:
        controller: FuzzyMotorController instance
    """
    error, delta_error, control = controller.get_membership_functions()

    error.view()
    plt.title('Error Membership Functions')
    plt.show()

    delta_error.view()
    plt.title('Delta Error Membership Functions')
    plt.show()

    control.view()
    plt.title('Control Signal Membership Functions')
    plt.show()


def plot_simulation_results(time_steps, actual_positions, targets, errors, control_signals, measured_positions=None):
    """
    Plot simulation results including position, error, and control signals.

    Args:
        time_steps: List of time values
        actual_positions: List of actual motor positions
        targets: List of target positions
        errors: List of position errors
        control_signals: List of control signal values
        measured_positions: List of encoder measurements (optional)
    """
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    axes[0, 0].plot(time_steps, actual_positions, 'b-', linewidth=2, label='Actual Position')
    if measured_positions is not None:
        axes[0, 0].plot(time_steps, measured_positions, 'c--', linewidth=1.5, alpha=0.7, label='Encoder Reading')
    axes[0, 0].plot(time_steps, targets, 'r--', linewidth=2, label='Target Position')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Position (degrees)')
    axes[0, 0].set_title('Motor Position vs Target (Closed-Loop with Encoder Feedback)')
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    axes[0, 1].plot(time_steps, errors, 'g-', linewidth=2)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Error (degrees)')
    axes[0, 1].set_title('Position Error Over Time')
    axes[0, 1].grid(True)

    axes[1, 0].plot(time_steps, control_signals, 'm-', linewidth=2)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Control Signal')
    axes[1, 0].set_title('Control Signal Over Time (Fuzzy Output)')
    axes[1, 0].grid(True)

    axes[1, 1].plot(errors, control_signals, 'c-', linewidth=1.5)
    axes[1, 1].set_xlabel('Error (degrees)')
    axes[1, 1].set_ylabel('Control Signal')
    axes[1, 1].set_title('Control Signal vs Error')
    axes[1, 1].grid(True)

    plt.tight_layout()
    plt.show()


def plot_control_surface(controller):
    """
    Plot 3D control surface showing control output for error and delta error.

    Args:
        controller: FuzzyMotorController instance
    """
    error_range = np.linspace(-180, 180, 30)
    delta_error_range = np.linspace(-50, 50, 30)
    x, y = np.meshgrid(error_range, delta_error_range)
    z = np.zeros_like(x)

    for i in range(30):
        for j in range(30):
            try:
                z[i, j] = controller.compute_control(x[i, j], y[i, j])
            except:
                z[i, j] = 0

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='viridis',
                          linewidth=0.4, antialiased=True)

    ax.set_xlabel('Error (degrees)')
    ax.set_ylabel('Delta Error (degrees/step)')
    ax.set_zlabel('Control Signal')
    ax.set_title('Fuzzy Control Surface')
    ax.view_init(30, 200)
    fig.colorbar(surf)
    plt.show()


def plot_final_summary(current_pos, target_pos, final_pos, steps):
    """
    Plot summary bar chart showing initial, target, and final positions.

    Args:
        current_pos: Initial position in degrees
        target_pos: Target position in degrees
        final_pos: Final achieved position in degrees
        steps: Number of simulation steps to converge
    """
    fig, ax = plt.subplots(figsize=(8, 6))

    positions = [current_pos, target_pos, final_pos]
    labels = ['Initial\nPosition', 'Target\nPosition', 'Final\nPosition']
    colors = ['blue', 'red', 'green']

    bars = ax.bar(labels, positions, color=colors, alpha=0.7, edgecolor='black', linewidth=2)

    for i, (bar, pos) in enumerate(zip(bars, positions)):
        height = bar.get_height()
        va = 'bottom' if pos >= 0 else 'top'
        y_pos = height if pos >= 0 else height
        ax.text(bar.get_x() + bar.get_width()/2., y_pos,
                f'{pos:.1f}°', ha='center', va=va, fontsize=12, fontweight='bold')

    ax.set_ylabel('Position (degrees)', fontsize=12)
    ax.set_title(f'Motor Position Control Summary\n(Converged in {steps} steps)', fontsize=14)
    ax.grid(True, axis='y', alpha=0.3)

    min_pos = min(positions)
    max_pos = max(positions)
    margin = (max_pos - min_pos) * 0.2 if max_pos != min_pos else 10
    ax.set_ylim([min_pos - margin, max_pos + margin])

    plt.tight_layout()
    plt.show()
