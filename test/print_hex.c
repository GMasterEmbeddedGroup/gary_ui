/**
*
*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

void print_hex(unsigned char * ptr, unsigned char length) {
    for (int i = length - 1; i >= 0; i--){
        printf("'%02x', ", *(ptr + i));
    }
    printf("\n");
}
void print_hex_real(unsigned char * ptr, unsigned char length) {
    for (int i = 0; i < length; i++){
        printf("'%02x', ", *(ptr + i));
    }
    printf("\n");
}


typedef struct
{
    uint8_t graphic_name[3];
    uint32_t operate_type:3;
    uint32_t graphic_type:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t radius:10;
    uint32_t end_x:11;
    uint32_t end_y:11;
} __attribute__((packed)) graphic_data_struct_t;


typedef struct
{
    uint8_t graphic_name[3];
    uint32_t operate_type:3;
    uint32_t graphic_type:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t value:24;
} __attribute__((packed)) graphic_float_struct_t;


int main() {
    graphic_data_struct_t val = {0};

    memcpy(&val.graphic_name, "abc", 3);
    val.operate_type = 1;
    val.graphic_type = 1;
    val.layer = 3;
    val.color = 2;
    val.start_angle = 10;
    val.end_angle = 0;
    val.width = 9;
    val.start_x = 10;
    val.start_y = 20;
    val.radius = 12;
    val.end_x = 900;
    val.end_y = 900;

    print_hex(&val, sizeof(graphic_data_struct_t) / sizeof(unsigned char));
    print_hex_real(&val, sizeof(graphic_data_struct_t) / sizeof(unsigned char));

    printf("\n\t---\n\n");

    graphic_float_struct_t f = {0};

    memcpy(&f.graphic_name, "flt", 3);
    f.operate_type = 1;
    f.graphic_type = 5;
    f.layer = 3;
    f.color = 1;
    f.start_angle = 10;
    f.end_angle = 4;
    f.width = 5;
    f.start_x = 100;
    f.start_y = 111;
    f.value = 1500;

//    print_hex(&f, sizeof(graphic_float_struct_t) / sizeof(unsigned char));
    print_hex_real(&f, sizeof(graphic_float_struct_t) / sizeof(unsigned char));
    return 0;
}
