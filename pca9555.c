#include <stdio.h>
#include "common/common.h"
#include "pca9555.h"

#define microseconds (1000)
#define milliseconds (1000 * microseconds)
#define seconds (1000LL * milliseconds)

PCA9555 new_PCA9555(HAL_PinOut pin, int a0, HAL_I2C i2c_hal, uint8_t hw_i2c_address)
{
    PCA9555 pca9555;
    zero(&pca9555);

    // Initialize the device structure
    pca9555._a0 = a0;
    pca9555._error = 0;
    pca9555._i2c_address = hw_i2c_address;                               
    pca9555._value_register = 0xFFFF;
    pca9555._base_configuration_register  = 0xFFFF;

    // Initialize the configuration arrays
    for (int i = 0; i < _num_of_channels; i++) {
        pca9555.config.GPIs[i] = 0;
        pca9555.config.GPOs[i] = 0;
    }

    // Initialize the read/write arrays
    for (int i = 0; i < _num_of_channels; i++) {
        pca9555.values.GPI_reads[i] = 0;
        pca9555.values.GPO_writes[i] = 0;
    }

    // i2c hal config
    pca9555.bus = i2c_hal;

    return pca9555;
}

error PCA9555_begin(PCA9555 *device)
{
    uint8_t test_address = NXP_CONFIG; // register address to be tested (found as command byte in data sheet)

    // Send a test write to the device to check its presence
    uint8_t buf[] = {test_address, 0xFF}; // set all pins as inputs. Writes to this register (R0) have no effect
    device->bus.tx(device->_i2c_address, buf, sizeof(buf), NULL, 0);

    if (device->_error < 0) {
        device->_error = device->_error; // Store the error code
        printf("Error initializing PCA9555. %d\n", device->_error);
        return 255; // Return false if the write operation failed
    } else {
        device->_error = 0; // No error
        return 0; // Return true if the write operation succeeded
    }
}

error PCA9555_pin_mode(PCA9555 *device, uint8_t pin, uint8_t IOMode)
{
    uint16_t val16;

    if (pin > 15)
    {
        device->_error = 255;
        return 255;
    }

    // IOMode = 0, pin is set to output mode
    if (IOMode == 0)
        // 1 << pin: This operation creates a bitmask where only the bit corresponding to pin is set to 1. 
        // For example, if pin is 3, 1 << 3 would be 00001000 in binary (Pins 7|6|5|4|3|2|1|0)

        // ~(1 << pin): The bitwise NOT operator (~) inverts the bits of the bitmask, 
        // so all bits are 1 except for the bit corresponding to pin, which is 0.
        // ~(1 << 3) would be 11110111 in binary

        // device->_base_configuration_register & ~(1 << pin): This operation uses the bitwise AND operator (&) 
        // to clear the bit corresponding to pin in the device->_base_configuration_register. 
        // If device->_base_configuration_register was initially 11111111 (all pins set to output), after this operation, 
        // it would become 11110111, with the pin bit cleared to 0

        val16 = device->_base_configuration_register & ~(1 << pin);

    // IOMode = 1, pin is set to input mode
    else
        // device->_base_configuration_register | (1 << pin): The bitwise OR operator (|) 
        // sets the bit corresponding to pin in the device->_base_configuration_register to 1. 
        // If the initial value of device->_base_configuration_register was 11110111, after this operation, 
        // it would become 11111111, with the pin bit set to 1.
        val16 = device->_base_configuration_register | (1 << pin);
        
    if (pin < 8)
        // NXP_CONFIG: Controls the lower 8 pins (pins 0 to 7).
        // val16 & 0x00FF: This operation masks val16 to extract only the lower 8 bits. 
        // The 0x00FF mask has all bits set to 1 except for the higher 8 bits. 
        // For example, if val16 is 0xABCD, val16 & 0x00FF would result in 0xCD.
        PCA9555_write_reg(device, device->_i2c_address, NXP_CONFIG, val16 & 0x00FF, false);
    else
        // NXP_CONFIG + 1: Controls the upper 8 pins (pins 8 to 15).
        // (val16 >> 8) & 0x00FF: This operation shifts val16 right by 8 bits, 
        // effectively moving the upper 8 bits of val16 into the lower 8 bits of the result. 
        // The & 0x00FF mask then extracts these lower 8 bits. For example, if val16 is 0xABCD, 
        // (val16 >> 8) & 0x00FF would result in 0xAB.
        PCA9555_write_reg(device, device->_i2c_address, NXP_CONFIG + 1, (val16 >> 8) & 0x00FF, false);
    
    if (device->_error)
        return 255;
    
    device->_base_configuration_register = val16;

    return 0;
}

error PCA9555_read(PCA9555 *device, uint8_t pin)
{
    uint8_t dataLow;
    uint8_t dataHigh;
    uint16_t inputData;

    // We will only process pins <= 15
    if (pin > 15) {
        device->_error = 255;
        return 255;
    }

    // Read the low byte of input data
    dataLow = I2C_get_value(device, device->_i2c_address, NXP_INPUT);
    if (device->_error < 0) {
        return device->_error; // Return the error if reading fails
    }

    // Read the high byte of input data
    dataHigh = I2C_get_value(device, device->_i2c_address, NXP_INPUT + 1);
    if (device->_error < 0) {
        return device->_error; // Return the error if reading fails
    }

    // Combine the two 8-bit values into a 16-bit value
    // dataHigh << 8: This operation shifts the dataHigh value 8 bits to the left. 
    // This effectively moves the bits of dataHigh into the higher byte of the 16-bit inputData.
    // | dataLow: This performs a bitwise OR operation between the shifted dataHigh and the dataLow. 
    // The dataLow value is placed in the lower byte of the 16-bit inputData (DATA 0 & DATA 1).
    inputData = (dataHigh << 8) | dataLow;

    // Mask the bit required and see if it is HIGH
    // 1 << pin: This shifts the value 1 left by pin positions. 
    // This operation creates a bitmask where only the bit at the position pin is set to 1, and all other bits are 0.
    // inputData & (1 << pin): This applies a bitwise AND operation between inputData and the bitmask created above. 
    // This isolates the bit at the position pin from inputData. If the bit at position pin in inputData is set to 1, 
    // the result will be non-zero; otherwise, it will be zero.
    if ((inputData & (1 << pin)) > 0) {
        // The bit is HIGH
        return HIGH;
    } else {
        return LOW;
    }
}

error PCA9555_write(PCA9555 *device, uint8_t pin, uint8_t state)
{
    uint16_t val16;

    if (pin > 15)
    {
        device->_error = 255;
        return 255;
    }

    if (state > 0)
        // 1 << pin: This operation creates a bitmask where only the bit at position pin is set to 1.
        // device->_value_register | (1 << pin): If state > 0, this performs a bitwise OR between 
        // device->_value_register and the bitmask created by 1 << pin. 
        // This operation sets the bit at the position pin to 1 (HIGH) without changing other bits in _value_register.
        val16 = device->_value_register | (1 << pin);
    else
        // device->_value_register & ~(1 << pin): If state is not greater than 0 (i.e., state is 0 or negative), 
        // this performs a bitwise AND between device->_value_register and the bitwise NOT of the bitmask 1 << pin. 
        // The bitmask ~(1 << pin) has all bits set to 1 except the bit at position pin, which is 0. 
        // This operation clears the bit at the position pin (sets it to 0) while preserving other bits.
        val16 = device->_value_register & ~(1 << pin);
    
    if (pin < 8)
        // val16 & 0x00FF: If pin is less than 8, this extracts the lower byte of val16 by performing a bitwise AND with 0x00FF. 
        // This operation masks out the higher byte, leaving only the lower byte.
        // This function call writes the lower byte of the updated value val16 to the GPIOA register of the PCA9555 device.
        PCA9555_write_GPIOA(device, val16 & 0x00FF);
    else
    // (val16 >> 8) & 0x00FF: If pin is 8 or higher, this extracts the higher byte of val16 by shifting 
    // val16 8 bits to the right and then masking with 0x00FF. The shift operation moves the higher byte 
    // to the lower byte position, and the mask operation isolates this lower byte.
    // This function call writes the higher byte of the updated value val16 to the GPIOB register of the PCA9555 device.
        PCA9555_write_GPIOB(device, (val16 >> 8) & 0x00FF);

    device->_value_register = val16;

    return 0;
}

error PCA9555_write_GPIOA(PCA9555 *device, uint8_t value)
{
    device->_error = PCA9555_write_reg(device, device->_i2c_address, NXP_OUTPUT, value, false);

    if (device->_error < 0)
        return 255;

    device->_value_register_low = value;

    return 0;
}

error PCA9555_write_GPIOB(PCA9555 *device, uint8_t value)
{
    device->_error = PCA9555_write_reg(device, device->_i2c_address, NXP_OUTPUT + 1, value, false);

    if (device->_error < 0)
        return 155;

    device->_value_register_high = value;

    return 0;
}

error PCA9555_write_GPIOAB(PCA9555 *device, uint16_t value)
{
    device->_error = PCA9555_write_reg(device, device->_i2c_address, NXP_OUTPUT, value, true);

    if (device->_error < 0)
        return 255;

    device->_value_register = value;

    return 0;
}

void PCA9555_get_pin_states(PCA9555 *device)
{
    device->_state_of_port_A_pins = I2C_get_value(device, device->_i2c_address, NXP_INPUT);
    device->_state_of_port_B_pins = I2C_get_value(device, device->_i2c_address, NXP_INPUT + 1);
    device->_state_of_pins = (device->_state_of_port_B_pins << 8) | device->_state_of_port_A_pins;
}

error PCA9555_get_pin_state(PCA9555 *device, uint8_t pin)
{
    // (1 << pin): This operation creates a bitmask where only the bit at position pin is set to 1. 
    // device->_state_of_pins & (1 << pin): This operation performs a bitwise AND between _state_of_pins and the bitmask. 
    // The result is non-zero (i.e., greater than 0) if the bit at position pin in _state_of_pins is 1 (HIGH).
    // If the bit at position pin is 0 (LOW), the result is 0.
    if ((device->_state_of_pins & (1 << pin)) > 0){
    // the bit is HIGH otherwise we would return a LOW value
        return HIGH;
    } else {
        return LOW;
    }
}

uint8_t I2C_get_value(PCA9555 *device, uint8_t address, uint8_t reg)
{
    int result;
    uint8_t buf[] = {reg};
    
    // Write the register address we want to read from
    result = device->bus.tx(device->_i2c_address, buf, sizeof(buf), NULL, 0);
    if (result < 0) {
        device->_error = result; // Store the error code
        return 0; // Return 0 as default error value
    }

    uint8_t data;
    // Read 1 byte of data from the register
    result = device->bus.tx(device->_i2c_address, NULL, 0, &data, sizeof(data));
    if (result < 0) {
        device->_error = result; // Store the error code
        return 0; // Return 0 as default error value
    }

    return data; // Return the read value
}

error PCA9555_write_reg(PCA9555 *device, uint8_t addr, uint8_t reg, uint16_t value, bool pair)
{
    // value & 0xFF: Contains the lower 8 bits of the value.
    // (value >> 8) & 0xFF: Contains the upper 8 bits of the value.
    uint8_t buf[3] = {reg, value & 0xFF, (value >> 8) & 0xFF};
    
    // pair ? 3 : 2: If pair is true, the buffer size is 3 (indicating a 16-bit write). 
    // If pair is false, the buffer size is 2 (indicating an 8-bit write).
    size_t buf_size = pair ? 3 : 2;
    device->bus.tx(device->_i2c_address, buf, buf_size, NULL, 0); 
    if (device->_error < 0)
        return device->_error;
    return 0; 
}