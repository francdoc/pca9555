// Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9555.pdf

#ifndef PCA9555_H
#define PCA9555_H

#include "common/common.h"

// 6.2.1 Command byte -> Table 4: 
#define NXP_INPUT      0x00 // Command 0 / Input port 0
#define NXP_OUTPUT     0x02 // Command 2 / Output port 0
#define NXP_INVERT     0x04 // Command 4 / Polarity Inversion port 0
#define NXP_CONFIG     0x06 // Command 6 / Configuration port 0

#define LOW            0x0
#define HIGH           0x1

#define _num_of_channels 8

/** enum with names of ports ED0 - ED15 */
enum {
    ED0, ED1, ED2 , ED3 , ED4 , ED5 , ED6 , ED7 ,
    ED8, ED9, ED10, ED11, ED12, ED13, ED14, ED15
};

/** enum with names of the ports as they're referred to on the TI datasheet */
enum {
    P00, P01, P02, P03, P04, P05, P06, P07,
    P10, P11, P12, P13, P14, P15, P16, P17,
};

struct configuration {
    bool GPIs[_num_of_channels]; // input pins
    bool GPOs[_num_of_channels]; // output pins
};

struct Read_write_values {
    bool GPI_reads[_num_of_channels];
    bool GPO_writes[_num_of_channels];
};

typedef struct {
    int _a0;
    uint8_t _state_of_port_A_pins;                // low order byte
    uint8_t _state_of_port_B_pins;                // high order byte
    uint8_t _base_configuration_register_low;     // low order byte
    uint8_t _base_configuration_register_high;    // high order byte
    uint8_t _value_register_low;                  // low order byte
    uint8_t _value_register_high;                 // high order byte
    uint8_t _i2c_address;   
    uint16_t _state_of_pins;                      // 16 bits presentation
    uint16_t _base_configuration_register;        // 16 bits presentation
    uint16_t _error;
    uint16_t _value_register;
    struct configuration config;
    struct Read_write_values values;
    HAL_I2C bus;
} PCA9555;

extern PCA9555 new_PCA9555(HAL_PinOut pin, int a0, HAL_I2C i2c_hal, uint8_t hw_i2c_address);
extern error PCA9555_begin(PCA9555 *device);
extern error PCA9555_pin_mode(PCA9555 *device, uint8_t pin, uint8_t IOMode);
extern error PCA9555_read(PCA9555 *device, uint8_t pin);
extern error PCA9555_write(PCA9555 *device, uint8_t pin, uint8_t state);
extern void PCA9555_get_pin_states(PCA9555 *device);
extern error PCA9555_get_pin_state(PCA9555 *device, uint8_t pin);
extern error PCA9555_write_GPIOA(PCA9555 *device, uint8_t value);
extern error PCA9555_write_GPIOB(PCA9555 *device, uint8_t value);
extern error PCA9555_write_GPIOAB(PCA9555 *device, uint16_t value);
extern error PCA9555_write_reg(PCA9555 *device, uint8_t addr, uint8_t reg, uint16_t value, bool pair);
extern uint8_t I2C_get_value(PCA9555 *device, uint8_t address, uint8_t reg);

#endif // IMPLEMENTATIONS_H