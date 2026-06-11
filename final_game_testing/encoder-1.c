#include "encoder.h"
 
PIO pio = pio0;
const uint sm = 0;
 
// encoder pins must be next to each other
#define ENCODER_FIRST_PIN 18
volatile int encoderOffset = 0;
 
void initEncoder(){
    // init the PIO for the encoder
 
    pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, sm, ENCODER_FIRST_PIN, 0);
}
 
int getEncoder(){
    // return the encoder counts
    return quadrature_encoder_get_count(pio, sm) - encoderOffset;
}
 
// set the offset to zero the encoder
void setEncoderToZero(){
    encoderOffset = quadrature_encoder_get_count(pio, sm);
}