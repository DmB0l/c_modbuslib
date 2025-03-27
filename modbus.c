#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "modbus.h"

// Вычисление CRC16
static uint16_t modbus_crc16(uint8_t* buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Инициализация устройства
ModbusDevice* modbus_init_device(uint16_t num_holding, uint16_t num_input,
                                 uint16_t num_coils, uint16_t num_discrete) {
    ModbusDevice* device = (ModbusDevice*)malloc(sizeof(ModbusDevice));

    device->num_holding_regs = num_holding;
    device->num_input_regs = num_input;
    device->num_coils = num_coils;
    device->num_discrete_inputs = num_discrete;

    device->holding_registers = (uint16_t*)calloc(num_holding, sizeof(uint16_t));
    device->input_registers = (uint16_t*)calloc(num_input, sizeof(uint16_t));
    device->coils = (bool*)calloc(num_coils, sizeof(bool));
    device->discrete_inputs = (bool*)calloc(num_discrete, sizeof(bool));

    return device;
}

// Освобождение памяти устройства
void modbus_free_device(ModbusDevice* device) {
    free(device->holding_registers);
    free(device->input_registers);
    free(device->coils);
    free(device->discrete_inputs);
    free(device);
}

// Создание Modbus запроса
int modbus_create_request(ModbusFrame* frame, uint8_t* buffer, uint16_t* length) {
    uint16_t pos = 0;

    buffer[pos++] = frame->slave_id;
    buffer[pos++] = frame->function_code;
    buffer[pos++] = (frame->address >> 8) & 0xFF;
    buffer[pos++] = frame->address & 0xFF;

    switch(frame->function_code) {
    case FC_READ_COILS:
    case FC_READ_DISCRETE_INPUTS:
    case FC_READ_HOLDING_REG:
    case FC_READ_INPUT_REG:
        buffer[pos++] = (frame->quantity >> 8) & 0xFF;
        buffer[pos++] = frame->quantity & 0xFF;
        break;

    case FC_WRITE_SINGLE_COIL:
        buffer[pos++] = (frame->quantity ? 0xFF : 0x00);
        buffer[pos++] = 0x00;
        break;

    case FC_WRITE_SINGLE_REG:
        buffer[pos++] = (frame->quantity >> 8) & 0xFF;
        buffer[pos++] = frame->quantity & 0xFF;
        break;

    case FC_WRITE_MULT_COILS: {
        buffer[pos++] = (frame->quantity >> 8) & 0xFF;
        buffer[pos++] = frame->quantity & 0xFF;
        uint8_t byte_count = (frame->quantity + 7) / 8;
        buffer[pos++] = byte_count;
        if (frame->data_length != byte_count || frame->data == NULL) {
            return MODBUS_ERR_VALUE;
        }
        memcpy(&buffer[pos], frame->data, byte_count);
        pos += byte_count;
        break;
    }

    case FC_WRITE_MULT_REG: {
        buffer[pos++] = (frame->quantity >> 8) & 0xFF;
        buffer[pos++] = frame->quantity & 0xFF;
        uint8_t byte_count = frame->quantity * 2;
        buffer[pos++] = byte_count;
        if (frame->data_length != byte_count || frame->data == NULL) {
            return MODBUS_ERR_VALUE;
        }
        memcpy(&buffer[pos], frame->data, byte_count);
        pos += byte_count;
        break;
    }
    }

    uint16_t crc = modbus_crc16(buffer, pos);
    buffer[pos++] = crc & 0xFF;
    buffer[pos++] = (crc >> 8) & 0xFF;

    *length = pos;
    return MODBUS_OK;
}

// Обработка запроса от master
int modbus_process_response(ModbusDevice* device, uint8_t* rx_buffer, uint16_t rx_length,
                            uint8_t* tx_buffer, uint16_t* tx_length) {
    if (rx_length < MODBUS_MIN_ADU_SIZE) return MODBUS_ERR_VALUE;

    // Проверка CRC
    uint16_t received_crc = (rx_buffer[rx_length-1] << 8) | rx_buffer[rx_length-2];
    uint16_t calculated_crc = modbus_crc16(rx_buffer, rx_length-2);
    if (received_crc != calculated_crc) return MODBUS_ERR_CRC;

    uint8_t slave_id = rx_buffer[0];
    uint8_t function = rx_buffer[1];
    uint16_t address = (rx_buffer[2] << 8) | rx_buffer[3];
    uint16_t pos = 0;

    tx_buffer[pos++] = slave_id;
    tx_buffer[pos++] = function;

    switch(function) {
    case FC_READ_COILS: {
        uint16_t quantity = (rx_buffer[4] << 8) | rx_buffer[5];
        if (address + quantity > device->num_coils) return MODBUS_ERR_ADDRESS;
        uint8_t byte_count = (quantity + 7) / 8;
        tx_buffer[pos++] = byte_count;
        for (uint16_t i = 0; i < quantity; i++) {
            if (i % 8 == 0) tx_buffer[pos + (i/8)] = 0;
            if (device->coils[address + i]) {
                tx_buffer[pos + (i/8)] |= (1 << (i % 8));
            }
        }
        pos += byte_count;
        break;
    }

    case FC_READ_DISCRETE_INPUTS: {
        uint16_t quantity = (rx_buffer[4] << 8) | rx_buffer[5];
        if (address + quantity > device->num_discrete_inputs) return MODBUS_ERR_ADDRESS;
        uint8_t byte_count = (quantity + 7) / 8;
        tx_buffer[pos++] = byte_count;
        for (uint16_t i = 0; i < quantity; i++) {
            if (i % 8 == 0) tx_buffer[pos + (i/8)] = 0;
            if (device->discrete_inputs[address + i]) {
                tx_buffer[pos + (i/8)] |= (1 << (i % 8));
            }
        }
        pos += byte_count;
        break;
    }

    case FC_READ_HOLDING_REG: {
        uint16_t quantity = (rx_buffer[4] << 8) | rx_buffer[5];
        if (address + quantity > device->num_holding_regs) return MODBUS_ERR_ADDRESS;
        tx_buffer[pos++] = quantity * 2;
        for (uint16_t i = 0; i < quantity; i++) {
            tx_buffer[pos++] = (device->holding_registers[address + i] >> 8) & 0xFF;
            tx_buffer[pos++] = device->holding_registers[address + i] & 0xFF;
        }
        break;
    }

    case FC_READ_INPUT_REG: {
        uint16_t quantity = (rx_buffer[4] << 8) | rx_buffer[5];
        if (address + quantity > device->num_input_regs) return MODBUS_ERR_ADDRESS;
        tx_buffer[pos++] = quantity * 2;
        for (uint16_t i = 0; i < quantity; i++) {
            tx_buffer[pos++] = (device->input_registers[address + i] >> 8) & 0xFF;
            tx_buffer[pos++] = device->input_registers[address + i] & 0xFF;
        }
        break;
    }

    case FC_WRITE_SINGLE_COIL: {
        if (address >= device->num_coils) return MODBUS_ERR_ADDRESS;
        uint16_t value = (rx_buffer[4] << 8) | rx_buffer[5];
        device->coils[address] = (value == 0xFF00); // FF00 = ON, 0000 = OFF
        memcpy(&tx_buffer[pos], &rx_buffer[2], 4);  // Эхо запроса
        pos += 4;
        break;
    }

    case FC_WRITE_SINGLE_REG: {
        if (address >= device->num_holding_regs) return MODBUS_ERR_ADDRESS;
        uint16_t value = (rx_buffer[4] << 8) | rx_buffer[5];
        device->holding_registers[address] = value;
        memcpy(&tx_buffer[pos], &rx_buffer[2], 4);  // Эхо запроса
        pos += 4;
        break;
    }

    case FC_WRITE_MULT_COILS: {
        uint16_t quantity = (rx_buffer[4] << 8) | rx_buffer[5];
        uint8_t byte_count = rx_buffer[6];
        if (address + quantity > device->num_coils) return MODBUS_ERR_ADDRESS;
        if (byte_count != ((quantity + 7) / 8)) return MODBUS_ERR_VALUE;
        for (uint16_t i = 0; i < quantity; i++) {
            uint8_t byte = rx_buffer[7 + (i / 8)];
            device->coils[address + i] = (byte & (1 << (i % 8))) != 0;
        }
        memcpy(&tx_buffer[pos], &rx_buffer[2], 4);  // Адрес и количество
        pos += 4;
        break;
    }

    case FC_WRITE_MULT_REG: {
        uint16_t quantity = (rx_buffer[4] << 8) | rx_buffer[5];
        uint8_t byte_count = rx_buffer[6];
        if (address + quantity > device->num_holding_regs) return MODBUS_ERR_ADDRESS;
        if (byte_count != (quantity * 2)) return MODBUS_ERR_VALUE;
        for (uint16_t i = 0; i < quantity; i++) {
            device->holding_registers[address + i] =
                (rx_buffer[7 + i*2] << 8) | rx_buffer[8 + i*2];
        }
        memcpy(&tx_buffer[pos], &rx_buffer[2], 4);  // Адрес и количество
        pos += 4;
        break;
    }

    default:
        return MODBUS_ERR_FUNCTION;
    }

    uint16_t crc = modbus_crc16(tx_buffer, pos);
    tx_buffer[pos++] = crc & 0xFF;
    tx_buffer[pos++] = (crc >> 8) & 0xFF;

    *tx_length = pos;
    return MODBUS_OK;
}

void print_hex(uint8_t* buffer, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
}

// Обработка ответа от slave устройства
uint16_t *modbus_parse_response(uint8_t *buffer, uint16_t length, uint16_t *value_count) {
    if (length < MODBUS_MIN_ADU_SIZE) {
        printf("Error: Response too short\n");
        *value_count = 0;
        return NULL;
    }

    uint8_t slave_id = buffer[0];
    uint8_t function = buffer[1];
    uint16_t crc = (buffer[length-1] << 8) | buffer[length-2];

    printf("Parsed Response:\n");
    printf("  Slave ID: %d\n", slave_id);
    printf("  Function Code: 0x%02X\n", function);

    if (function >= 0x80) {
        printf("  Error Code: 0x%02X\n", buffer[2]);
        printf("  CRC: 0x%04X\n", crc);
        *value_count = 0;
        return NULL;
    }

    uint16_t* result = NULL;
    *value_count = 0;

    switch(function) {
    case FC_READ_COILS:
    case FC_READ_DISCRETE_INPUTS: {
        uint8_t byte_count = buffer[2];
        uint16_t bit_count = byte_count * 8; // Максимальное число бит
        printf("  Byte Count: %d\n", byte_count);
        printf("  Values: ");

        // Выделяем память под значения (по одному биту на значение)
        result = (uint16_t*)malloc(bit_count * sizeof(uint16_t));
        *value_count = 0;

        for (uint8_t i = 0; i < byte_count; i++) {
            uint8_t byte = buffer[3 + i];
            for (int bit = 0; bit < 8 && (i * 8 + bit) < bit_count; bit++) {
                uint16_t value = (byte >> bit) & 1;
                result[*value_count] = value;
                printf("%d", value);
                if (i * 8 + bit + 1 < bit_count) printf(" ");
                (*value_count)++;
            }
        }
        printf("\n");
        break;
    }
    case FC_READ_HOLDING_REG:
    case FC_READ_INPUT_REG: {
        uint8_t byte_count = buffer[2];
        uint16_t reg_count = byte_count / 2;
        printf("  Byte Count: %d\n", byte_count);
        printf("  Register Values: ");

        // Выделяем память под значения регистров
        result = (uint16_t*)malloc(reg_count * sizeof(uint16_t));
        *value_count = reg_count;

        for (uint16_t i = 0; i < reg_count; i++) {
            uint16_t value = (buffer[3 + i*2] << 8) | buffer[4 + i*2];
            result[i] = value;
            printf("%d", value);
            if (i + 1 < reg_count) printf(", ");
        }
        printf("\n");
        break;
    }
    case FC_WRITE_SINGLE_COIL:
    case FC_WRITE_SINGLE_REG: {
        uint16_t address = (buffer[2] << 8) | buffer[3];
        uint16_t value = (buffer[4] << 8) | buffer[5];
        printf("  Address: %d\n", address);
        printf("  Value: ");
        if (function == FC_WRITE_SINGLE_COIL) {
            printf("%s\n", value == 0xFF00 ? "ON" : "OFF");
        } else {
            printf("%d\n", value);
        }
        // Ничего не возвращаем
        *value_count = 0;
        result = NULL;
        break;
    }
    case FC_WRITE_MULT_COILS:
    case FC_WRITE_MULT_REG: {
        uint16_t address = (buffer[2] << 8) | buffer[3];
        uint16_t quantity = (buffer[4] << 8) | buffer[5];
        printf("  Address: %d\n", address);
        printf("  Quantity: %d\n", quantity);
        // Ничего не возвращаем
        *value_count = 0;
        result = NULL;
        break;
    }
    default:
        printf("  Unknown function code\n");
        *value_count = 0;
        result = NULL;
        break;
    }
    printf("  CRC: 0x%04X\n", crc);

    return result;
}
