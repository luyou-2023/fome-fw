// [中文标题] local_version_holder.cpp
// 本地版本号持有者：用于检测数据是否发生变化的版本计数器

/**
 * @file	local_version_holder.cpp
 *
 * @date Mar 19, 2014
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "local_version_holder.h"

int LocalVersionHolder::getVersion() const {
	return localVersion;
}

bool LocalVersionHolder::isOld(int globalVersion) {
	if (globalVersion > localVersion) {
		localVersion = globalVersion;
		return true;
	}
	return false;
}
