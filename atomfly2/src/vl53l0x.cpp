#include "vl53l0x.hpp"

byte gbuf[16];

uint16_t bswap(byte b[]) {
    // Big Endian unsigned short to little endian unsigned short
    uint16_t val = ((b[0] << 8) & b[1]);
    return val;
}

uint16_t makeuint16(int lsb, int msb) {
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

void write_byte_data(byte data) {
    Wire1.beginTransmission(VL53L0X_address);
    Wire1.write(data);
    Wire1.endTransmission();
}

void write_byte_data_at(byte reg, byte data) {
    // write data word at address and register
    Wire1.beginTransmission(VL53L0X_address);
    Wire1.write(reg);
    Wire1.write(data);
    Wire1.endTransmission();
}

void write_word_data_at(byte reg, uint16_t data) {
    // write data word at address and register
    byte b0 = (data & 0xFF);
    byte b1 = ((data >> 8) && 0xFF);

    Wire1.beginTransmission(VL53L0X_address);
    Wire1.write(reg);
    Wire1.write(b0);
    Wire1.write(b1);
    Wire1.endTransmission();
}

byte read_byte_data() {
    Wire1.requestFrom(VL53L0X_address, 1);
    while (Wire1.available() < 1) delay(1);
    byte b = Wire1.read();
    return b;
}

byte read_byte_data_at(byte reg) {
    // write_byte_data((byte)0x00);
    write_byte_data(reg);
    Wire1.requestFrom(VL53L0X_address, 1);
    while (Wire1.available() < 1) delay(1);
    byte b = Wire1.read();
    return b;
}

uint16_t read_word_data_at(byte reg) {
    write_byte_data(reg);
    Wire1.requestFrom(VL53L0X_address, 2);
    while (Wire1.available() < 2) delay(1);
    gbuf[0] = Wire1.read();
    gbuf[1] = Wire1.read();
    return bswap(gbuf);
}

void read_block_data_at(byte reg, int sz) {
    int i = 0;
    write_byte_data(reg);
    Wire1.requestFrom(VL53L0X_address, sz);
    for (i = 0; i < sz; i++) {
        while (Wire1.available() < 1) delay(1);
        gbuf[i] = Wire1.read();
    }
}

uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg) {
    // Converts the encoded VCSEL period register value into the real
    // period in PLL clocks
    uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
    return vcsel_period_pclks;
}
