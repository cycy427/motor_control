/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 09:46:34
 * @LastEditTime: 2022-09-20 11:23:35
 */
#include "math_ops.h"
#include "math.h"

float my_fmaxf(float x, float y) {
    /// Returns maximum of x, y ///
    return (((x) > (y)) ? (x) : (y));
}

float my_minf(float x, float y) {
    /// Returns minimum of x, y ///
    return (((x) < (y)) ? (x) : (y));
}

float fmaxf3(float x, float y, float z) {
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}

float fminf3(float x, float y, float z) {
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}

void limit_norm(float *x, float *y, const float limit) {
    /// Scales the lenght of vector (x, y) to be <= limit ///
    const float norm = sqrtf(*x * *x + *y * *y);
    if (norm > limit) {
        *x = *x * limit / norm;
        *y = *y * limit / norm;
    }
}

int float_to_uint(const float x, const float x_min, const float x_max, const int bits) {
    //后面这个bits代表2的几次方
    /// Converts a float to an unsigned int, given range and number of bits ///
    const float span = x_max - x_min;
    const float offset = x_min;
    return (int) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}

float uint_to_float(const int x_int, const float x_min, const float x_max, const int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    const float span = x_max - x_min;
    const float offset = x_min;
    return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

union F32 {
    float v_float;
    unsigned int v_int;
    unsigned char buf[4];
} f32;

void float32_to_float16(const float *float32, unsigned short int *float16) {
    unsigned short int temp = 0;
    f32.v_float = *float32;
    //	*float16 = ((f32.v_int & 0x7fffffff) >> 13) - (0x38000000 >> 13);
    //  *float16 |= ((f32.v_int & 0x80000000) >> 16);
    temp = (f32.buf[3] & 0x7F) << 1 | ((f32.buf[2] & 0x80) >> 7);
    temp -= 112;
    *float16 = temp << 10 | (f32.buf[2] & 0x7F) << 3 | f32.buf[1] >> 5;
    *float16 |= ((f32.v_int & 0x80000000) >> 16);
}

void float16_to_float32(const unsigned short int *float16, float *float32) {
    //	f32.v_int=*float16;
    //	f32.v_int = ((f32.v_int & 0x7fff) << 13) + 0x7f000000;
    //  f32.v_int |= ((*float16 & 0x8000) << 16);
    //	*float32=f32.v_float;
    unsigned short int temp2 = 0;
    f32.v_int = 0;
    temp2 = (((*float16 & 0x7C00) >> 10) + 112);
    f32.buf[3] = temp2 >> 1;
    f32.buf[2] = ((temp2 & 0x01) << 7) | (*float16 & 0x03FC) >> 3;
    f32.buf[1] = (*float16 & 0x03) << 6;
    f32.v_int |= ((*float16 & 0x8000) << 16);
    *float32 = f32.v_float;
}

double clamping(const double x, const double x_min, const double x_max) {
    // 处理无效输入范围（x_max <= x_min 时返回 0）
    if (x_max <= x_min) return 0;
    // 分配空间，并初始化
    double clamped_x = x;
    // 将输入值钳位到 [x_min, x_max] 范围内
    if (x < x_min) clamped_x = x_min;
    else if (x > x_max) clamped_x = x_max;
    // 线性映射到无符号整型范围，并截断小数部分
    return clamped_x;
}
