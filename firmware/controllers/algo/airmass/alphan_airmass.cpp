/* [Alpha-N空气量]
 * 基于节气门位置和发动机转速估算进气量。
 * 适用于无法准确测量 MAP 的应用场景（如大节气门、高重叠角凸轮轴）。
 */

#include "pch.h"

#include "alphan_airmass.h"

AirmassResult AlphaNAirmass::getAirmass(float rpm, bool postState) {
	auto tps = Sensor::get(SensorType::Tps1);

	if (!tps.Valid) {
		// We are fully reliant on TPS - if the TPS fails, stop the engine.
		return {};
	}

	// In this case, VE directly describes the cylinder filling relative to the ideal
	float ve = getVe(rpm, tps.Value, postState);

	// optionally use real IAT instead of fixed air temperature
	constexpr float standardIat = 20.0f; // std atmosphere temperature
	float iat = engineConfiguration->alphaNUseIat ? Sensor::get(SensorType::Iat).value_or(standardIat) : standardIat;

	float iatK = iat + 273;

	// TODO: should this be barometric pressure and/or temperature compensated?
	mass_t airmass = getAirmassImpl(
			ve,
			101.325f, // std atmosphere pressure
			iatK);

	return {airmass, tps.Value};
}
