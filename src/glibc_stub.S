/*
 * glibc 兼容桩 —— 为 Radxa glibc 2.36 提供 __isoc23_strtoul
 *
 * GCC 13 (Ubuntu 24.04) 的 glibc 头文件将 strtoul 重定向到 __isoc23_strtoul@GLIBC_2.38，
 * 但 Radxa 的 glibc 2.36 没有此符号。汇编桩直接尾调用实际 strtoul。
 */

.section .text
.globl  __isoc23_strtoul
.type   __isoc23_strtoul, @function
__isoc23_strtoul:
    b   strtoul
.size   __isoc23_strtoul, .-__isoc23_strtoul

.globl  __isoc23_strtoull
.type   __isoc23_strtoull, @function
__isoc23_strtoull:
    b   strtoull
.size   __isoc23_strtoull, .-__isoc23_strtoull
