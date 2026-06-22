// [GCC版本检查]
// 编译时检查GCC版本是否满足最低要求(>=11.3.1)。
// 新版本GCC修复了ARM嵌入式编译的关键bug,低于此版本可能导致固件异常。
// 非生产构建(EFI_UNIT_TEST/EFI_SIMULATOR)跳过此检查。
// This file asserts that the compiler is appropriate for rusEFI use.

// non-MCU builds are significantly more tolerant
#if EFI_PROD_CODE

#define GCC_VERSION ((__GNUC__ * 100) + (__GNUC_MINOR__ * 10) + (__GNUC_PATCHLEVEL__))

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#pragma message("GCC is " STR(__GNUC__) "." STR(__GNUC_MINOR__) "." STR(__GNUC_PATCHLEVEL__))

// Firmware builds require at least GCC 11.3.1
#if (GCC_VERSION < 1131)
#error "GCC compiler >= 11.3.1 required"
#endif

#endif
