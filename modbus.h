#ifndef MODBUS_H
#define MODBUS_H

#include <stdint.h>
#include <stdbool.h>

// Определение констант Modbus
#define MODBUS_MAX_ADU_SIZE     256
#define MODBUS_MIN_ADU_SIZE     4
#define MODBUS_CRC_SIZE         2

// Коды функций Modbus
#define FC_READ_COILS           0x01
#define FC_READ_DISCRETE_INPUTS 0x02
#define FC_READ_HOLDING_REG     0x03
#define FC_READ_INPUT_REG       0x04
#define FC_WRITE_SINGLE_COIL    0x05
#define FC_WRITE_SINGLE_REG     0x06
#define FC_WRITE_MULT_COILS     0x0F
#define FC_WRITE_MULT_REG       0x10

// Коды ошибок
#define MODBUS_OK               0
#define MODBUS_ERR_CRC          1
#define MODBUS_ERR_FUNCTION     2
#define MODBUS_ERR_ADDRESS      3
#define MODBUS_ERR_VALUE        4

// Структура Modbus фрейма
typedef struct {
    uint8_t slave_id;
    uint8_t function_code;
    uint16_t address;
    uint16_t quantity;
    uint8_t* data;
    uint16_t data_length;
} ModbusFrame;

// Структура для хранения регистров устройства
typedef struct {
    uint16_t* holding_registers;
    uint16_t* input_registers;
    bool* coils;
    bool* discrete_inputs;
    uint16_t num_holding_regs;
    uint16_t num_input_regs;
    uint16_t num_coils;
    uint16_t num_discrete_inputs;
} ModbusDevice;

// Прототипы функций
ModbusDevice* modbus_init_device(uint16_t num_holding, uint16_t num_input,
                                 uint16_t num_coils, uint16_t num_discrete);
void modbus_free_device(ModbusDevice* device);
int modbus_create_request(ModbusFrame* frame, uint8_t* buffer, uint16_t* length);
int modbus_process_response(ModbusDevice* device, uint8_t* rx_buffer, uint16_t rx_length,
                            uint8_t* tx_buffer, uint16_t* tx_length);
uint16_t* modbus_parse_response(uint8_t* buffer, uint16_t length, uint16_t* value_count);
void print_hex(uint8_t* buffer, uint16_t length);

#endif // MODBUS_H
