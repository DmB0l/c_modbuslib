#include <stdio.h>
#include <stdlib.h>
#include "modbus.h"

int main() {
    // Инициализация устройства
    ModbusDevice* device = modbus_init_device(100, 100, 100, 100);

    uint8_t request_buffer[MODBUS_MAX_ADU_SIZE];
    uint8_t response_buffer[MODBUS_MAX_ADU_SIZE];
    uint16_t req_length, resp_length;

    // 5. Write Single Coil (FC 05) - Установка coil по адресу 20 в 1
    ModbusFrame fc05 = {
        .slave_id = 1,
        .function_code = FC_WRITE_SINGLE_COIL,
        .address = 20,
        .quantity = 1,  // 1 для ON, 0 для OFF
        .data = NULL,
        .data_length = 0
    };
    modbus_create_request(&fc05, request_buffer, &req_length);
    printf("FC05 Request: ");
    print_hex(request_buffer, req_length);
    modbus_process_response_from_master(device, request_buffer, req_length, response_buffer, &resp_length);
    printf("FC05 Response: ");
    print_hex(response_buffer, resp_length);
    printf("\n");

    // 1. Read Coils (FC 01) - Чтение 5 coils начиная с адреса 10
    ModbusFrame fc01 = {
        .slave_id = 1,
        .function_code = FC_READ_COILS,
        .address = 10,
        .quantity = 5,
        .data = NULL,
        .data_length = 0
    };
    modbus_create_request(&fc01, request_buffer, &req_length);
    printf("FC01 Request: ");
    print_hex(request_buffer, req_length);
    modbus_process_response_from_master(device, request_buffer, req_length, response_buffer, &resp_length);
    printf("FC01 Response: ");
    print_hex(response_buffer, resp_length);
    printf("\n");

    // 2. Read Discrete Inputs (FC 02) - Чтение 8 дискретных входов с адреса 0
    ModbusFrame fc02 = {
        .slave_id = 1,
        .function_code = FC_READ_DISCRETE_INPUTS,
        .address = 0,
        .quantity = 8,
        .data = NULL,
        .data_length = 0
    };
    modbus_create_request(&fc02, request_buffer, &req_length);
    printf("FC02 Request: ");
    print_hex(request_buffer, req_length);
    modbus_process_response_from_master(device, request_buffer, req_length, response_buffer, &resp_length);
    printf("FC02 Response: ");
    print_hex(response_buffer, resp_length);
    printf("\n");

    // 3. Read Holding Registers (FC 03) - Чтение 3 регистров с адреса 5
    ModbusFrame fc03 = {
        .slave_id = 1,
        .function_code = FC_READ_HOLDING_REG,
        .address = 5,
        .quantity = 3,
        .data = NULL,
        .data_length = 0
    };
    modbus_create_request(&fc03, request_buffer, &req_length);
    printf("FC03 Request: ");
    print_hex(request_buffer, req_length);
    modbus_process_response_from_master(device, request_buffer, req_length, response_buffer, &resp_length);
    printf("FC03 Response: ");
    print_hex(response_buffer, resp_length);
    printf("\n");

    // 4. Read Input Registers (FC 04) - Чтение 2 регистров с адреса 1
    ModbusFrame fc04 = {
        .slave_id = 1,
        .function_code = FC_READ_INPUT_REG,
        .address = 1,
        .quantity = 2,
        .data = NULL,
        .data_length = 0
    };
    modbus_create_request(&fc04, request_buffer, &req_length);
    printf("FC04 Request: ");
    print_hex(request_buffer, req_length);
    modbus_process_response_from_master(device, request_buffer, req_length, response_buffer, &resp_length);
    printf("FC04 Response: ");
    print_hex(response_buffer, resp_length);
    printf("\n");

    // 6. Write Single Register (FC 06) - Запись значения 1234 в регистр 15
    ModbusFrame fc06 = {
        .slave_id = 1,
        .function_code = FC_WRITE_SINGLE_REG,
        .address = 15,
        .quantity = 1234,
        .data = NULL,
        .data_length = 0
    };
    modbus_create_request(&fc06, request_buffer, &req_length);
    printf("FC06 Request: ");
    print_hex(request_buffer, req_length);
    modbus_process_response_from_master(device, request_buffer, req_length, response_buffer, &resp_length);
    printf("FC06 Response: ");
    print_hex(response_buffer, resp_length);
    printf("\n");

    // 15. Write Multiple Coils (FC 0F) - Запись 10 coils начиная с адреса 20
    uint8_t coils_data[] = {0xCD, 0x01}; // 11001101 00000001 (10 бит)
    ModbusFrame fc0f = {
        .slave_id = 1,
        .function_code = FC_WRITE_MULT_COILS,
        .address = 20,
        .quantity = 10,
        .data = coils_data,
        .data_length = 2
    };
    modbus_create_request(&fc0f, request_buffer, &req_length);
    printf("FC0F Request: ");
    print_hex(request_buffer, req_length);
    modbus_process_response_from_master(device, request_buffer, req_length, response_buffer, &resp_length);
    printf("FC0F Response: ");
    print_hex(response_buffer, resp_length);
    printf("\n");

    // 16. Write Multiple Registers (FC 10) - Запись 3 регистров с адреса 10
    uint8_t regs_data[] = {0x00, 0x0A, 0x00, 0x0B, 0x00, 0x0C}; // 10, 11, 12
    ModbusFrame fc10 = {
        .slave_id = 1,
        .function_code = FC_WRITE_MULT_REG,
        .address = 10,
        .quantity = 3,
        .data = regs_data,
        .data_length = 6
    };
    modbus_create_request(&fc10, request_buffer, &req_length);
    printf("FC10 Request: ");
    print_hex(request_buffer, req_length);
    modbus_process_response_from_master(device, request_buffer, req_length, response_buffer, &resp_length);
    printf("FC10 Response: ");
    print_hex(response_buffer, resp_length);
    printf("\n");

    // Проверка результата записи coils
    ModbusFrame fc01_check = {
        .slave_id = 1,
        .function_code = FC_READ_COILS,
        .address = 20,
        .quantity = 10,
        .data = NULL,
        .data_length = 0
    };
    modbus_create_request(&fc01_check, request_buffer, &req_length);
    printf("FC01 Check Request: ");
    print_hex(request_buffer, req_length);
    modbus_process_response_from_master(device, request_buffer, req_length, response_buffer, &resp_length);
    printf("FC01 Check Response: ");
    print_hex(response_buffer, resp_length);
    printf("\n");

    // Проверка результата записи регистров
    ModbusFrame fc03_check = {
        .slave_id = 1,
        .function_code = FC_READ_HOLDING_REG,
        .address = 10,
        .quantity = 3,
        .data = NULL,
        .data_length = 0
    };
    modbus_create_request(&fc03_check, request_buffer, &req_length);
    printf("FC03 Check Request: ");
    print_hex(request_buffer, req_length);
    modbus_process_response_from_master(device, request_buffer, req_length, response_buffer, &resp_length);
    printf("FC03 Check Response: ");
    print_hex(response_buffer, resp_length);
    printf("\n");


    uint16_t value_count;
    uint16_t* values = modbus_process_response_from_slave(response_buffer, resp_length, &value_count);

    if (values != NULL) {
        printf("Returned values: ");
        for (uint16_t i = 0; i < value_count; i++) {
            printf("%d ", values[i]);
        }
        printf("\n");
        free(values); // Не забудьте освободить память!
    } else {
        printf("No values returned (write operation or error)\n");
    }




    // Освобождение памяти
    modbus_free_device(device);
    return 0;
}
