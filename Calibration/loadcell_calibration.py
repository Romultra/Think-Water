"""
Load Cell Calibration Analysis Script
Reads calibration data and computes the calibration factor using linear regression.
"""

import csv
import numpy as np
import matplotlib.pyplot as plt


def load_calibration_data(filepath):
    """Load calibration data from CSV file."""
    reference_weights = []
    raw_values = []

    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                weight = float(row['reference_g'])
                raw = float(row['raw_value'])
                reference_weights.append(weight)
                raw_values.append(raw)
            except (ValueError, KeyError):
                continue

    return np.array(reference_weights), np.array(raw_values)


def compute_calibration(weights, raw_values):
    """
    Compute calibration factor using linear regression.

    Model: raw_value = scale * weight + offset
    Therefore: weight = (raw_value - offset) / scale

    Returns:
        scale: The scale factor (raw units per gram)
        offset: The zero offset (raw value at 0 grams)
        r_squared: R-squared value of the fit
    """
    # Linear regression: raw = scale * weight + offset
    coefficients = np.polyfit(weights, raw_values, 1)
    scale = coefficients[0]  # slope (raw units per gram)
    offset = coefficients[1]  # intercept (raw value at 0g)

    # Calculate R-squared
    predicted = np.polyval(coefficients, weights)
    ss_res = np.sum((raw_values - predicted) ** 2)
    ss_tot = np.sum((raw_values - np.mean(raw_values)) ** 2)
    r_squared = 1 - (ss_res / ss_tot)

    return scale, offset, r_squared


def plot_calibration(weights, raw_values, scale, offset, r_squared):
    """Plot calibration data and fitted line."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Plot 1: Raw data with fitted line
    ax1 = axes[0]
    ax1.scatter(weights, raw_values, color='blue', label='Measurements', alpha=0.7, s=50)

    # Fitted line
    x_fit = np.linspace(min(weights), max(weights), 100)
    y_fit = scale * x_fit + offset
    ax1.plot(x_fit, y_fit, 'r-', linewidth=2, label=f'Linear fit (R²={r_squared:.6f})')

    ax1.set_xlabel('Reference Weight (g)', fontsize=12)
    ax1.set_ylabel('Raw ADC Value', fontsize=12)
    ax1.set_title('Load Cell Calibration Curve', fontsize=14)
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot 2: Residuals
    ax2 = axes[1]
    predicted = scale * weights + offset
    residuals = raw_values - predicted
    residuals_grams = residuals / scale  # Convert to grams

    ax2.scatter(weights, residuals_grams, color='green', alpha=0.7, s=50)
    ax2.axhline(y=0, color='r', linestyle='--', linewidth=1)
    ax2.set_xlabel('Reference Weight (g)', fontsize=12)
    ax2.set_ylabel('Residual (g)', fontsize=12)
    ax2.set_title('Residuals (Error in grams)', fontsize=14)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('calibration_plot.png', dpi=150)
    plt.show()


def main():
    # Load data
    print("Loading calibration data...")
    weights, raw_values = load_calibration_data('CALIB.CSV')
    print(f"Loaded {len(weights)} measurements")

    # Compute calibration
    print("\nComputing calibration factor...")
    scale, offset, r_squared = compute_calibration(weights, raw_values)

    # Print results
    print("\n" + "=" * 60)
    print("CALIBRATION RESULTS")
    print("=" * 60)
    print(f"\nLinear model: raw_value = {scale:.4f} * weight + {offset:.4f}")
    print(f"\nCalibration parameters:")
    print(f"  Scale factor:  {scale:.4f} raw units/gram")
    print(f"  Zero offset:   {offset:.4f} raw units")
    print(f"  R-squared:     {r_squared:.6f}")

    print(f"\nTo convert raw value to grams:")
    print(f"  weight_g = (raw_value - ({offset:.4f})) / ({scale:.4f})")
    print(f"  weight_g = (raw_value + {-offset:.4f}) / {scale:.4f}")

    # For firmware use (often need integer scale factor)
    print("\n" + "-" * 60)
    print("FOR FIRMWARE USE:")
    print("-" * 60)
    print(f"  #define LOADCELL_OFFSET  {int(round(offset))}")
    print(f"  #define LOADCELL_SCALE   {scale:.4f}f")
    print(f"\n  // Or as integer math (multiply result by 100 for 2 decimal places):")
    print(f"  // weight_cg = (raw_value - OFFSET) * 100 / SCALE")
    print(f"  #define LOADCELL_SCALE_INT  {int(round(scale))}")

    # Calculate max error
    predicted = scale * weights + offset
    residuals = raw_values - predicted
    max_error_raw = np.max(np.abs(residuals))
    max_error_g = max_error_raw / abs(scale)
    avg_error_g = np.mean(np.abs(residuals)) / abs(scale)

    print("\n" + "-" * 60)
    print("ACCURACY:")
    print("-" * 60)
    print(f"  Max error:     {max_error_g:.3f} g")
    print(f"  Average error: {avg_error_g:.3f} g")

    # Plot
    print("\nGenerating calibration plot...")
    plot_calibration(weights, raw_values, scale, offset, r_squared)
    print("Plot saved to 'calibration_plot.png'")


if __name__ == '__main__':
    main()
