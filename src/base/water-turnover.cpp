#include <stdint.h>

/**
 * Predict daily water turnover (mL/day)
 * Simplified for Switzerland household use
 * Based on Li et al., Nutrients 2023 (Science 2022 model)
 *
 * Fixed assumptions:
 * - Non-athlete
 * - Mean Swiss household altitude
 * - High HDI country
 * - Lightly active older adult
 *
 * @param bodyWeightKg     Body weight in kg
 * @param sex              0 = female, 1 = male
 * @param humidityPercent  Relative humidity (%)
 * @param ageYears         Age in years
 * @param temperatureC    Ambient temperature (°C)
 *
 * @return Water turnover in mL/day
 */
float predictWaterTurnoverCH(
    float bodyWeightKg,
    uint8_t sex,
    float humidityPercent,
    float ageYears,
    float temperatureC
) {
    // ---- Fixed constants ----
    const float PHYSICAL_ACTIVITY_LEVEL = 1.5f;
    const float ALTITUDE_METERS = 500.0f;
    const uint8_t HDI_CATEGORY = 0; // Switzerland = high HDI

    float waterTurnover =
          1076.0f * PHYSICAL_ACTIVITY_LEVEL
        + 14.34f * bodyWeightKg
        + 374.9f * sex
        + 5.823f * humidityPercent
        + 104.6f * HDI_CATEGORY
        + 0.4726f * ALTITUDE_METERS
        - 0.3529f * ageYears * ageYears
        + 24.78f * ageYears
        + 1.865f * temperatureC * temperatureC
        - 19.66f * temperatureC
        - 713.1f;

    // Safety clamp
    if (waterTurnover < 0.0f) {
        waterTurnover = 0.0f;
    }

    return waterTurnover;
}
