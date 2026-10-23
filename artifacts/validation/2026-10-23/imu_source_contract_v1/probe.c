/* PC fake-register contract check. No device, ROS, clock mapping or control. */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "lsm6dsox_reg.h"

struct bus { uint8_t data[256]; int32_t error; unsigned reads; unsigned writes; };
static int32_t read_register(void *handle, uint8_t address, uint8_t *data, uint16_t size) {
    struct bus *b = handle;
    assert((unsigned)address + size <= sizeof(b->data));
    ++b->reads;
    if (b->error) { if (size) data[0] = 0xff; return b->error; }
    memcpy(data, b->data + address, size);
    return 0;
}
static int32_t write_register(void *handle, uint8_t address, const uint8_t *data, uint16_t size) {
    struct bus *b = handle;
    assert((unsigned)address + size <= sizeof(b->data));
    ++b->writes;
    if (b->error) return b->error;
    memcpy(b->data + address, data, size);
    return 0;
}
static void vector_bytes(struct bus *b, uint8_t address, const int16_t values[3]) {
    for (unsigned i=0; i<3; ++i) {
        uint16_t bits = (uint16_t)values[i];
        b->data[address+2*i] = bits & 255u;
        b->data[address+2*i+1] = bits >> 8;
    }
}
int main(void) {
    struct bus b = {0};
    stmdev_ctx_t ctx = {0};
    ctx.handle = &b; ctx.read_reg = read_register; ctx.write_reg = write_register;
    float_t (*converters[])(int16_t) = {
        lsm6dsox_from_fs2_to_mg, lsm6dsox_from_fs4_to_mg,
        lsm6dsox_from_fs8_to_mg, lsm6dsox_from_fs16_to_mg,
        lsm6dsox_from_fs125_to_mdps, lsm6dsox_from_fs250_to_mdps,
        lsm6dsox_from_fs500_to_mdps, lsm6dsox_from_fs1000_to_mdps,
        lsm6dsox_from_fs2000_to_mdps};
    const double factors[] = {.061, .122, .244, .488, 4.375, 8.75, 17.5, 35., 70.};
    unsigned unit_checks = 0;
    for (unsigned f=0; f<9; ++f) {
        for (int32_t raw=INT16_MIN; raw<=INT16_MAX; ++raw) {
            double got = converters[f]((int16_t)raw), expected = raw*factors[f];
            assert(fabs(got-expected) <= 1e-6*fmax(1., fabs(expected)));
            ++unit_checks;
        }
    }
    /* Full-scale setting is explicit; these examples are not a chosen calibration. */
    double gyro_si = lsm6dsox_from_fs500_to_mdps(1000) / 1000. * acos(-1.) / 180.;
    double accel_si = lsm6dsox_from_fs4_to_mg(1000) / 1000. * 9.80665;
    assert(fabs(gyro_si - .30543261909900765) < 1e-12);
    assert(fabs(accel_si - 1.1964113) < 1e-6);

    const int16_t values[3] = {INT16_MIN, -1, INT16_MAX};
    vector_bytes(&b, LSM6DSOX_OUTX_L_G, values);
    vector_bytes(&b, LSM6DSOX_OUTX_L_A, values);
    int16_t gyro[3] = {0}, accel[3] = {0};
    assert(lsm6dsox_angular_rate_raw_get(&ctx, gyro) == 0);
    assert(lsm6dsox_acceleration_raw_get(&ctx, accel) == 0);
    assert(memcmp(gyro, values, sizeof gyro) == 0 && memcmp(accel, values, sizeof accel) == 0);
    const uint32_t stamps[] = {0, 1, 32767, 32768, 65535, 65536, UINT32_MAX};
    for (unsigned i=0; i<sizeof stamps/sizeof stamps[0]; ++i) {
        for (unsigned k=0; k<4; ++k) b.data[LSM6DSOX_TIMESTAMP0+k] = (stamps[i]>>(8*k)) & 255u;
        uint32_t got = 0;
        assert(lsm6dsox_timestamp_raw_get(&ctx, &got) == 0 && got == stamps[i]);
    }
    uint32_t stamp = 123;
    b.error = -7;
    assert(lsm6dsox_angular_rate_raw_get(&ctx, gyro) == -7);
    assert(lsm6dsox_acceleration_raw_get(&ctx, accel) == -7);
    assert(lsm6dsox_timestamp_raw_get(&ctx, &stamp) == -7);
    assert(memcmp(gyro, values, sizeof gyro) == 0 && memcmp(accel, values, sizeof accel) == 0 && stamp == 123);
    b.error = 0;
    assert(lsm6dsox_block_data_update_set(&ctx, 1) == 0);
    uint8_t bdu = 0;
    assert(lsm6dsox_block_data_update_get(&ctx, &bdu) == 0 && bdu == 1);
    /* The int16_t helper cannot represent the entire uint32_t timestamp register. */
    double helper_at_negative_one = lsm6dsox_from_lsb_to_nsec(-1);
    uint64_t full_register_ns = (uint64_t)UINT32_MAX * UINT64_C(25000);
    assert(helper_at_negative_one == -25000.);
    assert(full_register_ns == UINT64_C(107374182375000));
    printf("{\"scope\":\"pinned_vendor_source_fake_registers_only\",\"unit_vector_comparisons\":%u,"
           "\"signed_vectors_ok\":true,\"timestamp_vectors\":7,\"read_error_output_unchanged\":true,"
           "\"bdu_readback_ok\":true,\"gyro_1000_fs500_rad_s\":%.15g,\"accel_1000_fs4_mps2\":%.15g,"
           "\"helper_minus_one_ns\":%.0f,\"uint32_max_times_25000_ns\":%llu,"
           "\"hardware_tested\":false,\"acquisition_timing_verified\":false}\n",
           unit_checks, gyro_si, accel_si, helper_at_negative_one, (unsigned long long)full_register_ns);
    return 0;
}
