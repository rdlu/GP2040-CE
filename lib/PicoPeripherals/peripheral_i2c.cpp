#include <cstdio>
#include <pico.h>
#include "peripheral_i2c.h"

PeripheralI2C::PeripheralI2C() {
    mutex_init(&_mutex);
#ifdef PICO_DEFAULT_I2C_INSTANCE

#if PICO_SDK_VERSION_MAJOR >= 2
    _I2C = PICO_DEFAULT_I2C_INSTANCE();
#else
    _I2C = PICO_DEFAULT_I2C_INSTANCE;
#endif

    _SDA = PICO_DEFAULT_I2C_SDA_PIN;
    _SCL = PICO_DEFAULT_I2C_SCL_PIN;
    _Speed = DEFAULT_SPEED;
#endif
}

// The display (core1) and web config / input add-ons (core0) can share one I2C
// block. The SDK transfer functions reprogram the controller (enable/TAR) and
// poll/clear its abort flags, so two cores using it at once corrupt each
// other's transfers and an untimed i2c_read_blocking() can spin forever.
// Core0 callers get priority over core1 (display) so inputs are not starved.
void PeripheralI2C::lock() {
    if (get_core_num() == 0) {
        _inputPending.store(true, std::memory_order_release);
        mutex_enter_blocking(&_mutex);
        _inputPending.store(false, std::memory_order_release);
        return;
    }

    while (true) {
        while (_inputPending.load(std::memory_order_acquire)) tight_loop_contents();
        mutex_enter_blocking(&_mutex);
        if (!_inputPending.load(std::memory_order_acquire)) return;
        mutex_exit(&_mutex);
    }
}

void PeripheralI2C::unlock() {
    mutex_exit(&_mutex);
}

void PeripheralI2C::setConfig(uint8_t block, int8_t sda, int8_t scl, uint32_t speed) {
    if ((block < NUM_I2CS) && (sda > -1) && (scl > -1)) {
        _I2C = _hardwareBlocks[block];
        _SDA = sda;
        _SCL = scl;
        _Speed = speed;
        configured = true;
        setup();
    } else {
        // currently not supported
    }
}

void PeripheralI2C::setup() {
    if ((_SDA + 2 * i2c_hw_index(_I2C))%4 != 0) return;
    if ((_SCL + 3 + 2 * i2c_hw_index(_I2C))%4 != 0) return;

    i2c_init(_I2C, _Speed);
    gpio_set_function(_SDA, GPIO_FUNC_I2C);
    gpio_set_function(_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(_SDA);
    gpio_pull_up(_SCL);

    // reset the bus before using it
    clear();
}

int16_t PeripheralI2C::read(uint8_t address, uint8_t *data, uint16_t len, bool isBlock) {
    if ((_exclusiveAddress > -1) && (_exclusiveAddress != address)) return -1;

    lock();
    int16_t result = i2c_read_blocking(_I2C, address, data, len, isBlock);
    unlock();
#ifdef DEBUG_PERIPHERALI2C
    printf("PeripheralI2C::write %d:%d (blocking? %d)\n", address, len, isBlock);
    for (int i = 0; i < len; i++) {
        printf("%02x ", data[i]);
    }
    printf("\nResult: %d\n", result);
    printf("-----\n");
#endif    
    return result;
}

int16_t PeripheralI2C::readRegister(uint8_t address, uint8_t reg, uint8_t *data, uint16_t len) {
    if ((_exclusiveAddress > -1) && (_exclusiveAddress != address)) return -1;

    int16_t registerCheck;
    lock();
    registerCheck = i2c_write_blocking(_I2C, address, &reg, 1, true);
    if (registerCheck >= 0) {
        registerCheck = i2c_read_blocking(_I2C, address, data, len, false);
    }
    unlock();
    return (registerCheck >= 0);
}

int16_t PeripheralI2C::write(uint8_t address, uint8_t *data, uint16_t len, bool isBlock) {
    if ((_exclusiveAddress > -1) && (_exclusiveAddress != address)) return -1;

#ifdef DEBUG_PERIPHERALI2C
    printf("PeripheralI2C::write %d:%d (blocking? %d)\n", address, len, isBlock);
    for (int i = 0; i < len; i++) {
        printf("%02x ", data[i]);
    }
#endif
    lock();
    int16_t result = i2c_write_blocking(_I2C, address, data, len, isBlock);
    unlock();
#ifdef DEBUG_PERIPHERALI2C
    printf("\nResult: %d\n", result);
    printf("-----\n");
#endif
    return result;
}

uint8_t PeripheralI2C::test(uint8_t address) {
    uint8_t data;
    
    // TODO: Revert to i2c_read_blocking when we have I2C resolved
    // int16_t ret = i2c_read_blocking(_I2C, address, &data, 1, false);
    lock();
    absolute_time_t test_timeout = make_timeout_time_ms(100);
    int16_t ret = i2c_read_blocking_until(_I2C, address, &data, 1, false, test_timeout);
    unlock();
    return (ret >= 0);
}

void PeripheralI2C::clear() {
    // reset the bus
    test(0xFF);
}

std::map<uint8_t,bool> PeripheralI2C::scan() {
    std::map<uint8_t,bool> result;

    bool found[1 << 7] = {};

    // Hold the bus for the whole scan (~128 short probes, a few ms at 400 kHz)
    // instead of interleaving with display transfers, and bound every probe so
    // a stuck bus cannot hang the core that also services USB/RNDIS.
    lock();
    for (uint8_t addr = 0x08; addr < 0x78; ++addr) { // skip reserved addresses
        uint8_t rxdata;
        int ret = i2c_read_blocking_until(_I2C, addr, &rxdata, 1, false, make_timeout_time_ms(5));
        found[addr] = (ret >= 0);
    }
    unlock();

    // allocate only after releasing the bus
    for (uint8_t addr = 0; addr < (1 << 7); ++addr) {
        if (found[addr]) result.insert({addr, true});
    }

#ifdef DEBUG_PERIPHERALI2C
    printf("%d\n", result.size());
#endif

    return result;
}