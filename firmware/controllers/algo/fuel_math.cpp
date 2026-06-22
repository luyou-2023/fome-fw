/**
 * @file	fuel_math.cpp
 * @brief	Fuel amount calculation logic
 *
 * [燃油计算核心]
 * 本文件实现了ECU燃油控制的核心算法:
 * 1. 空气量计算(Speed-Density/MAF/Alpha-N)
 * 2. 目标空燃比查表
 * 3. 各种修正项(温度/气压/加速/起动)
 * 4. 减速断油(DFCO)
 */
 *
 * @date May 27, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 *
 * This file is part of rusEfi - see http://rusefi.com
 *
 * rusEfi is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * rusEfi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "pch.h"

#include "airmass.h"
#include "alphan_airmass.h"
#include "maf_airmass.h"
#include "speed_density_airmass.h"
#include "fuel_math.h"
#include "fuel_computer.h"
#include "injector_model.h"
#include "speed_density.h"
#include "speed_density_base.h"
#include "lua_hooks.h"

/* MAP估算表: 当直接MAP传感器不可用时
 * 通过TPS和RPM估算进气压力(备份模式)
 * 3D查表: TPS_Bins × RPM_Bins → 估算MAP值
 */
static mapEstimate_Map3D_t mapEstimationTable;

#if EFI_ENGINE_CONTROL

/* ===== 起动燃油量计算 =====
 * 起动时燃油需求远高于正常运行,原因:
 * 1. 冷气缸壁使燃油凝结,实际参与燃烧的仅一部分
 * 2. 低温燃油雾化不良
 * 3. 需要快速建立气缸壁油膜
 *
 * baseFuel: 基础燃油量(克)
 * revolutionCounterSinceStart: 起动开始后的发动机转数
 *   随着转数增加,油膜逐渐建立,燃油量递减
 */
float getCrankingFuel3(float baseFuel, uint32_t revolutionCounterSinceStart) {

	float baseCrankingFuel;
	/* useRunningMathForCranking: 某些发动机在起动时
	 * 直接使用运行燃油计算,不单独配置起动油量
	 */
	if (engineConfiguration->useRunningMathForCranking) {
		baseCrankingFuel = baseFuel;
	} else {
		// parameter is in milligrams, convert to grams
		baseCrankingFuel = engineConfiguration->cranking.baseFuel * 0.001f;
	}

	/* 燃油系数 = f(起动转数)
	 * 第1转: 最大加浓(建立油膜)
	 * 第N转: 逐渐衰减到接近正常值
	 * 2D查表: 转数→衰减系数
	 */
	engine->engineState.crankingFuel.durationCoefficient =
			interpolate2d(revolutionCounterSinceStart, config->crankingCycleBins, config->crankingCycleCoef);

	/**
	 * Cranking fuel is different depending on engine coolant temperature
	 * If the sensor is failed, use 20 deg C
	 */
	auto clt = Sensor::get(SensorType::Clt).value_or(20);
	auto e0Mult = interpolate2d(clt, config->crankingFuelBins, config->crankingFuelCoef);

	/* 加浓系数检查: 如果<0.1意味着配置表有严重问题
	 * 打印警告但继续运行(使用可能错误的值也比不能起动好)
	 */
	bool alreadyWarned = false;
	if (e0Mult <= 0.1f) {
		warning(ObdCode::CUSTOM_ERR_ZERO_E0_MULT, "zero e0 multiplier");
		alreadyWarned = true;
	}

	/* 乙醇燃料(E85/E100)的起动加浓曲线与汽油不同
	 * 乙醇在低温下蒸发性更差,需要更多加浓
	 * flexCranking开启且乙醇传感器存在时,在汽油和E85曲线间插值
	 */
	if (engineConfiguration->flexCranking && Sensor::hasSensor(SensorType::FuelEthanolPercent)) {
		auto e85Mult = interpolate2d(clt, config->crankingFuelBins, config->crankingFuelCoefE100);

		if (e85Mult <= 0.1f) {
			warning(ObdCode::CUSTOM_ERR_ZERO_E85_MULT, "zero e85 multiplier");
			alreadyWarned = true;
		}

		// If failed flex sensor, default to 50% E
		auto flex = Sensor::get(SensorType::FuelEthanolPercent).value_or(50);

		/* interpolateClamped: 在汽油系数(e0Mult)和E85系数(e85Mult)之间
		 * 根据乙醇含量(flex%)线性插值
		 * 0%乙醇 → 汽油系数; 85%乙醇 → E85系数
		 */
		engine->engineState.crankingFuel.coolantTemperatureCoefficient =
				interpolateClamped(0, e0Mult, 85, e85Mult, flex);
	} else {
		engine->engineState.crankingFuel.coolantTemperatureCoefficient = e0Mult;
	}

	/* TPS修正: 脚踏油门起动时增加燃油
	 * 用于: 清缸模式(清除淹缸)/手动加浓辅助
	 * TPS故障时系数=1(不修正)
	 */
	auto tps = Sensor::get(SensorType::DriverThrottleIntent);
	engine->engineState.crankingFuel.tpsCoefficient =
			tps.Valid ? interpolate2d(tps.Value, config->crankingTpsBins, config->crankingTpsCoef)
					  : 1; // in case of failed TPS, don't correct.

	/* 最终起动油量 = 基础油量 × 三系数乘积
	 * 三系数: 转数衰减 × 水温加浓 × TPS修正
	 */
	floatms_t crankingFuel = baseCrankingFuel * engine->engineState.crankingFuel.durationCoefficient *
							 engine->engineState.crankingFuel.coolantTemperatureCoefficient *
							 engine->engineState.crankingFuel.tpsCoefficient;

	engine->engineState.crankingFuel.fuel = crankingFuel * 1000;

	// don't re-warn for zero fuel when we already warned for a more specific problem
	if (!alreadyWarned && crankingFuel <= 0) {
		warning(ObdCode::CUSTOM_ERR_ZERO_CRANKING_FUEL, "Cranking fuel value %f", crankingFuel);
	}
	return crankingFuel;
}

/* ===== 运行燃油修正 =====
 * 对基础燃油量应用所有修正系数
 * 修正系数为乘积关系(所有修正串联):
 * final = base × baro × iat × clt × postCranking × als × launch
 * 注意: 乘积意味着各修正相互独立——改变一个不影响其他
 * 这也是最常用的修正组合方式
 */
float getRunningFuel(float baseFuel) {
	ScopePerf perf(PE::GetRunningFuel);

	float iatCorrection = engine->fuelComputer.running.intakeTemperatureCoefficient;
	float cltCorrection = engine->fuelComputer.running.coolantTemperatureCoefficient;
	float postCrankingFuelCorrection = engine->fuelComputer.running.postCrankingFuelCorrection;
	float baroCorrection = engine->engineState.baroCorrection;

	/* NaN断言: 如果任何修正项为NaN(未初始化/计算错误)
	 * 断言失败但返回0(非致命),继续运行
	 */
	efiAssert(ObdCode::CUSTOM_ERR_ASSERT, !std::isnan(iatCorrection), "NaN iatCorrection", 0);
	efiAssert(ObdCode::CUSTOM_ERR_ASSERT, !std::isnan(cltCorrection), "NaN cltCorrection", 0);
	efiAssert(ObdCode::CUSTOM_ERR_ASSERT, !std::isnan(postCrankingFuelCorrection), "NaN postCrankingFuelCorrection", 0);

	float correction = baroCorrection * iatCorrection * cltCorrection * postCrankingFuelCorrection;

#if EFI_ANTILAG_SYSTEM
	/* ALS燃油修正: 百分比值,如10%加浓 → 乘以1.10
	 * ALS在松油门时加浓+推迟点火,利用排气管内燃烧保持涡轮转速
	 */
	correction *= (1 + engine->antilagController.fuelALSCorrection / 100);
#endif /* EFI_ANTILAG_SYSTEM */

#if EFI_LAUNCH_CONTROL
	/* 弹射起步燃油修正: 通常在弹射时加浓以冷却排气温度
	 */
	correction *= engine->launchController.getFuelCoefficient();
#endif

	float runningFuel = baseFuel * correction;

	efiAssert(ObdCode::CUSTOM_ERR_ASSERT, !std::isnan(runningFuel), "NaN runningFuel", 0);

	// Publish output state
	engine->fuelComputer.running.baseFuel = baseFuel * 1000;
	engine->fuelComputer.totalFuelCorrection = correction;
	engine->fuelComputer.running.fuel = runningFuel * 1000;

	return runningFuel;
}

/* ===== 空气量模型选择 =====
 * Speed-Density: 通过MAP+VE表计算空气量(主流)
 *   airMass = MAP × VE × displacement / (R × IAT)
 * MAF: 直接测量空气质量流量(精度高但压损大)
 * Alpha-N: TPS+RPM查表(适用于高凸轮轴重叠)
 * Lua: 用户自定义模型(灵活)
 */
static SpeedDensityAirmass sdAirmass(nullptr, mapEstimationTable);
static MafAirmass mafAirmass;
static AlphaNAirmass alphaNAirmass;

AirmassModelBase* getAirmassModel(engine_load_mode_e mode) {
	switch (mode) {
		case LM_SPEED_DENSITY:
			return &sdAirmass;
		case LM_REAL_MAF:
			return &mafAirmass;
		case LM_ALPHA_N:
			return &alphaNAirmass;
#if EFI_LUA
		case LM_LUA:
			return &(getLuaAirmassModel());
#endif
#if EFI_UNIT_TEST
		case LM_MOCK:
			return engine->mockAirmassModel;
#endif
		default:
			firmwareError(ObdCode::CUSTOM_ERR_ASSERT, "Invalid airmass mode %d", engineConfiguration->fuelAlgorithm);
			return nullptr;
	}
}

/* ===== 每缸基础燃油质量 =====
 * 计算流程:
 * 1. 选择空气量模型获取气缸空气量
 * 2. 标准化充气效率用于输出显示
 * 3. 空气量 → 目标空燃比 → 燃油质量
 * 4. 应用全局修正系数
 *
 * rpm参数: 用于计算发动机周期时间
 * 输出: 每缸每循环燃油质量(克)
 */
static float getBaseFuelMass(float rpm) {
	ScopePerf perf(PE::GetBaseFuel);

	// airmass modes - get airmass first, then convert to fuel
	auto model = getAirmassModel(engineConfiguration->fuelAlgorithm);
	efiAssert(ObdCode::CUSTOM_ERR_ASSERT, model != nullptr, "Invalid airmass mode", 0.0f);

	/* getAirmass(rpm, true): 第二个参数表示使用进气温度修正
	 * 返回结果包含:
	 *   CylinderAirmass: 每缸空气量(克)
	 *   EngineLoadPercent: 发动机负荷百分比(用于查表)
	 */
	auto airmass = model->getAirmass(rpm, true);

	// Plop some state for others to read
	/* 标准化充气效率: 当前空气量/理论100%空气量 × 100
	 * 理论100%: 标准温度压力下(20°C,101.325kPa)气缸完全充满
	 * 用于TunerStudio显示和日志记录
	 */
	float normalizedCylinderFilling = 100 * airmass.CylinderAirmass / getStandardAirCharge();
	engine->fuelComputer.sdAirMassInOneCylinder = airmass.CylinderAirmass;
	engine->fuelComputer.normalizedCylinderFilling = normalizedCylinderFilling;
	engine->engineState.fuelingLoad = airmass.EngineLoadPercent;
	engine->engineState.ignitionLoad =
			engine->fuelComputer.getLoadOverride(airmass.EngineLoadPercent, engineConfiguration->ignOverrideMode);

	/* 空气流量估算(kg/h): 用于其他子系统
	 * gramPerCycle = 单缸空气量 × 缸数(每周期总进气)
	 * gramPerMs = gramPerCycle / 周期时间(毫秒)
	 * kg/h = gramPerMs × 3600000/1000
	 */
	auto gramPerCycle = airmass.CylinderAirmass * engineConfiguration->cylindersCount;
	auto gramPerMs = rpm == 0 ? 0 : gramPerCycle / getEngineCycleDuration(rpm);

	// convert g/s -> kg/h
	engine->engineState.airflowEstimate = gramPerMs * 3600000 /* milliseconds per hour */ / 1000 /* grams per kg */;
	;

	/* getCycleFuel: 核心转换——空气量→燃油质量
	 * 内部计算: fuelMass = airMass / targetAFR
	 * targetAFR来自3D表(RPM×负荷→Lambda目标)
	 * 如果启用了闭环, targetAFR还会被氧传感器反馈修正
	 */
	float baseFuelMass = engine->fuelComputer.getCycleFuel(airmass.CylinderAirmass, rpm, airmass.EngineLoadPercent);

	// Fudge it by the global correction factor
	baseFuelMass *= engineConfiguration->globalFuelCorrection;
	engine->engineState.baseFuel = baseFuelMass;

	if (std::isnan(baseFuelMass)) {
		// todo: we should not have this here but https://github.com/rusefi/rusefi/issues/1690
		return 0;
	}

	return baseFuelMass;
}

/* ===== 喷油正时计算 =====
 * 决定每个气缸在曲轴哪个角度开始喷油
 * 通常应在进气门关闭前结束喷油,让燃油有时间雾化混合
 * 3D查表: RPM × 负荷 → 喷油起始角度(°BTDC)
 * 典型值: 进气上止点前300-360°
 * 大负荷时需要更早喷油(更多燃油需要更长雾化时间)
 */
angle_t getInjectionOffset(float rpm, float load) {
	if (std::isnan(rpm)) {
		return 0; // error already reported
	}

	if (std::isnan(load)) {
		return 0; // error already reported
	}

	angle_t value = interpolate3d(config->injectionPhase, config->injPhaseLoadBins, load, config->injPhaseRpmBins, rpm);

	/* 查表失败保护: 如果表还未初始化(如配置重置过程中)
	 * 返回0度(上止点喷油)作为安全默认
	 */
	if (std::isnan(value)) {
		// we could be here while resetting configuration for example
		// huh? what? when do we have RPM while resetting configuration? is that CI edge case? shall we fix CI?
		warning(ObdCode::CUSTOM_ERR_6569, "phase map not ready");
		return 0;
	}

	/* wrapAngle: 确保角度在[0,720)范围内
	 * 不同喷油器配置可能导致超出范围的角度
	 */
	angle_t result = value;
	wrapAngle(result, "inj offset#2", ObdCode::CUSTOM_ERR_6553);
	return result;
}

/**
 * Number of injections using each injector per engine cycle
 * @see getNumberOfSparks
 */
/* ===== 每周期喷油次数 =====
 * 不同喷油模式下每个喷油器每个发动机周期的动作次数:
 * 顺序(Sequential):    1次 — 每次只喷一个气缸,最精确
 * 批次(Batch):        2次 — 每半圈喷一半气缸
 * 同时(Simultaneous): 缸数次 — 所有气缸同时喷油(起动时常用)
 * 单点(SinglePoint):  缸数次 — 节气门上方一个喷油器
 */
int getNumberOfInjections(injection_mode_e mode) {
	switch (mode) {
		case IM_SIMULTANEOUS:
		case IM_SINGLE_POINT:
			return engineConfiguration->cylindersCount;
		case IM_BATCH:
			return 2;
		case IM_SEQUENTIAL:
			return 1;
		default:
			firmwareError(ObdCode::CUSTOM_ERR_INVALID_INJECTION_MODE, "Unexpected injection_mode_e %d", mode);
			return 1;
	}
}

/* 喷射模式持续时间乘数: 补偿不同模式下喷油器开启时间
 * 模式影响的是总燃油量分配到几次喷射中:
 * 同时: 所有缸同时喷 → 单次喷射时间为每缸的1/缸数
 * 顺序: 逐缸喷射 → 单次喷射时间=每缸需求
 * 批次: 两组交替 → 单次喷射时间为每缸的1/2
 */
float getInjectionModeDurationMultiplier(injection_mode_e mode) {
	switch (mode) {
		case IM_SIMULTANEOUS: {
			auto cylCount = engineConfiguration->cylindersCount;

			if (cylCount == 0) {
				// we can end up here during configuration reset
				return 0;
			}

			return 1.0f / cylCount;
		}
		case IM_SEQUENTIAL:
		case IM_SINGLE_POINT:
			return 1;
		case IM_BATCH:
			return 0.5f;
		default:
			firmwareError(ObdCode::CUSTOM_ERR_INVALID_INJECTION_MODE, "Unexpected injection_mode_e %d", mode);
			return 0;
	}
}

percent_t getInjectorDutyCycle(float rpm) {
	auto mode = getCurrentInjectionMode();
	floatms_t totalInjectiorAmountPerCycle = engine->engineState.injectionDuration * getNumberOfInjections(mode);
	floatms_t engineCycleDuration = getEngineCycleDuration(rpm);
	return 100 * totalInjectiorAmountPerCycle / engineCycleDuration;
}

percent_t getInjectorDutyCycleStage2(float rpm) {
	auto mode = getCurrentInjectionMode();
	floatms_t totalInjectiorAmountPerCycle = engine->engineState.injectionDurationStage2 * getNumberOfInjections(mode);
	floatms_t engineCycleDuration = getEngineCycleDuration(rpm);
	return 100 * totalInjectiorAmountPerCycle / engineCycleDuration;
}

static float getCycleFuelMass(bool isCranking, float baseFuelMass) {
	if (isCranking) {
		return getCrankingFuel(baseFuelMass);
	} else {
		return getRunningFuel(baseFuelMass);
	}
}

/**
 * @returns	Mass of each individual fuel injection, in grams
 *     in case of single point injection mode the amount of fuel into all cylinders, otherwise the amount for one
 * cylinder
 */
/* ===== 单次循环喷油量(顶层入口) =====
 * 这是燃油计算的总入口,在快速回调(250Hz)中调用
 * 流程:
 * 1. 计算基础燃油量(空气量→目标AFR→燃油质量)
 * 2. 应用起动/运行修正
 * 3. 减速断油(DFCO)判断
 * 4. 准备喷油器模型(流量+无效时间)
 * 5. TPS加速补偿
 *
 * 返回值: 单次喷射的燃油质量(克)
 */
float getCycleInjectionMass(float rpm, bool isCranking) {
	ScopePerf perf(PE::GetInjectionDuration);

#if EFI_SHAFT_POSITION_INPUT
	// Always update base fuel - some cranking modes use it
	float baseFuelMass = getBaseFuelMass(rpm);

	/* 选择起动或运行燃油修正路径
	 * isCranking标志由RPM阈值和起动状态决定
	 */
	float cycleFuelMass = getCycleFuelMass(isCranking, baseFuelMass);
	efiAssert(ObdCode::CUSTOM_ERR_ASSERT, !std::isnan(cycleFuelMass), "NaN cycleFuelMass", 0);

	/* DFCO(减速断油):
	 * 松油门且RPM高于阈值时完全切断燃油
	 * 省油+降排气温度
	 * 恢复时渐进恢复喷油防止熄火
	 */
	if (engine->module<DfcoController>()->cutFuel()) {
		// If decel fuel cut, zero out fuel
		cycleFuelMass = 0;
	}

	/* 准备喷油器模型:
	 * prepare()根据当前电池电压计算无效时间(dead time)
	 * 无效时间 = 喷油器从通电到完全打开的机械延迟
	 * 电压低→电磁力弱→开启慢→无效时间长
	 * 分段喷射时有主/副两套喷油器
	 */
	engine->module<InjectorModelPrimary>()->prepare();

	if (engineConfiguration->enableStagedInjection) {
		engine->module<InjectorModelSecondary>()->prepare();
	}

	/* TPS加速补偿:
	 * 节气门突然打开→进气量立即增加
	 * 但进气道壁面油膜需要时间响应→混合气瞬时变稀
	 * getTpsEnrichment()返回额外喷油时间(ms)
	 * 基于TPS变化率查表
	 */
	floatms_t tpsAccelEnrich = engine->module<TpsAccelEnrichment>()->getTpsEnrichment();
	efiAssert(ObdCode::CUSTOM_ERR_ASSERT, !std::isnan(tpsAccelEnrich), "NaN tpsAccelEnrich", 0);
	engine->engineState.tpsAccelEnrich = tpsAccelEnrich;

	/* TPS加速表存储为时间(ms)而非质量(克)的历史原因
	 * 需要转换回质量以便与cycleFuelMass相加
	 * 使用当前喷油器模型(含电压修正)做转换
	 */
	float tpsFuelMass = engine->module<InjectorModelPrimary>()->getFuelMassForDuration(tpsAccelEnrich);

	return cycleFuelMass + tpsFuelMass;
#else
	return 0;
#endif
}

/**
 * @brief	Initialize fuel map data structure
 * @note this method has nothing to do with fuel map VALUES - it's job
 * is to prepare the fuel map data structure for 3d interpolation
 */
void initFuelMap() {
	mapEstimationTable.init(config->mapEstimateTable, config->mapEstimateTpsBins, config->mapEstimateRpmBins);
}

/**
 * @brief Engine warm-up fuel correction.
 */
float getCltFuelCorrection() {
	const auto clt = Sensor::get(SensorType::Clt);

	if (!clt) {
		return 1; // this error should be already reported somewhere else, let's just handle it
	}

	return interpolate2d(clt.Value, config->cltFuelCorrBins, config->cltFuelCorr);
}

float getIatFuelCorrection() {
	const auto iat = Sensor::get(SensorType::Iat);

	if (!iat) {
		return 1; // this error should be already reported somewhere else, let's just handle it
	}

	return interpolate2d(iat.Value, config->iatFuelCorrBins, config->iatFuelCorr);
}

float getBaroCorrection() {
	if (Sensor::hasSensor(SensorType::BarometricPressure)) {
		// Default to 1atm if failed
		float pressure = Sensor::get(SensorType::BarometricPressure).value_or(101.325f);

		float correction = interpolate3d(
				config->baroCorrTable,
				config->baroCorrPressureBins,
				pressure,
				config->baroCorrRpmBins,
				Sensor::getOrZero(SensorType::Rpm));

		if (std::isnan(correction) || correction < 0.01) {
			warning(ObdCode::OBD_Barometric_Press_Circ_Range_Perf, "Invalid baro correction %f", correction);
			return 1;
		}

		return correction;
	} else {
		return 1;
	}
}

percent_t getFuelALSCorrection(float rpm) {
#if EFI_ANTILAG_SYSTEM
	if (engine->antilagController.isAntilagCondition) {
		float throttleIntent = Sensor::getOrZero(SensorType::DriverThrottleIntent);
		auto AlsFuelAdd = interpolate3d(
				config->ALSFuelAdjustment,
				config->alsFuelAdjustmentLoadBins,
				throttleIntent,
				config->alsFuelAdjustmentrpmBins,
				rpm);
		return AlsFuelAdd;
	} else
#endif /* EFI_ANTILAG_SYSTEM */
	{
		return 0;
	}
}

#if EFI_ENGINE_CONTROL
/**
 * @return Duration of fuel injection while craning
 */
float getCrankingFuel(float baseFuel) {
	return getCrankingFuel3(baseFuel, engine->rpmCalculator.getRevolutionCounterSinceStart());
}

/**
 * Standard cylinder air charge - 100% VE at standard temperature, grams per cylinder
 *
 * Should we bother caching 'getStandardAirCharge' result or can we afford to run the math every time we calculate fuel?
 */
float getStandardAirCharge() {
	float totalDisplacement = engineConfiguration->displacement;
	float cylDisplacement = totalDisplacement / engineConfiguration->cylindersCount;

	// Calculation of 100% VE air mass in g/cyl - 1 cylinder filling at 1.204/L
	// 101.325kpa, 20C
	return idealGasLaw(cylDisplacement, 101.325f, 273.15f + 20.0f);
}

float getCylinderFuelTrim(size_t cylinderNumber, float rpm, float fuelLoad) {
	auto trimPercent = interpolate3d(
			config->fuelTrims[cylinderNumber].table, config->fuelTrimLoadBins, fuelLoad, config->fuelTrimRpmBins, rpm);

	// Convert from percent +- to multiplier
	// 5% -> 1.05
	return (100 + trimPercent) / 100;
}

static Hysteresis stage2Hysteresis;

float getStage2InjectionFraction(float rpm, float load) {
	if (!engineConfiguration->enableStagedInjection) {
		return 0;
	}

	float frac = 0.01f * interpolate3d(
								 config->injectorStagingTable,
								 config->injectorStagingLoadBins,
								 load,
								 config->injectorStagingRpmBins,
								 rpm);

	// don't allow very small fraction, with some hysteresis
	if (!stage2Hysteresis.test(frac, 0.1, 0.03)) {
		return 0;
	}

	// Clamp to 90%
	if (frac > 0.9) {
		frac = 0.9;
	}

	return frac;
}

#endif
#endif
