#include "controller.h"
#include "../config.h"
#include "../hwconfig.h"
#include "core/bus.h"
#include <Arduino.h>

extern HWConfig hw_config;
static uint8_t (*_controllerRead)() = nullptr;

uint8_t controllerRead()
{
    return _controllerRead();
}

bool isDownPressed(CONTROLLER button)
{
    return (controllerRead() & (uint8_t)button) != 0;
}

static uint8_t gpioRead()
{
    uint8_t state = 0x00;
    if (digitalRead(A_BUTTON) == LOW) state |= (uint8_t)CONTROLLER::A;
    if (digitalRead(B_BUTTON) == LOW) state |= (uint8_t)CONTROLLER::B;
    if (digitalRead(SELECT_BUTTON) == LOW) state |= (uint8_t)CONTROLLER::Select;
    if (digitalRead(START_BUTTON) == LOW) state |= (uint8_t)CONTROLLER::Start;
    if (digitalRead(UP_BUTTON) == LOW) state |= (uint8_t)CONTROLLER::Up;
    if (digitalRead(DOWN_BUTTON) == LOW) state |= (uint8_t)CONTROLLER::Down;
    if (digitalRead(LEFT_BUTTON) == LOW) state |= (uint8_t)CONTROLLER::Left;
    if (digitalRead(RIGHT_BUTTON) == LOW) state |= (uint8_t)CONTROLLER::Right;

    return state;
}

static uint8_t NESControllerRead()
{
    uint8_t state = 0x00;
    digitalWrite(CONTROLLER_NES_LATCH, HIGH);
    delayMicroseconds(12);
    digitalWrite(CONTROLLER_NES_LATCH, LOW);
    delayMicroseconds(6);

    for (int i = 0; i < 8; i++)
    {
        if (digitalRead(CONTROLLER_NES_DATA) == LOW) state |= (1 << i);
        digitalWrite(CONTROLLER_NES_CLK, LOW);
        delayMicroseconds(6);
        digitalWrite(CONTROLLER_NES_CLK, HIGH);
        delayMicroseconds(6);
    }

    return state;
}

static uint8_t SNESControllerRead_old()
{
    // SNES bits
    // 0 - B
    // 1 - Y
    // 2 - Select
    // 3 - Start
    // 4 - Up
    // 5 - Down
    // 6 - Left
    // 7 - Right
    // 8 - A
    // 9 - X
    // 10 - L
    // 11 - R

    uint8_t state = 0x00;
    uint16_t snes_state = 0x0000;
    digitalWrite(CONTROLLER_SNES_LATCH, HIGH);
    delayMicroseconds(12);
    digitalWrite(CONTROLLER_SNES_LATCH, LOW);
    delayMicroseconds(6);

    for (int i = 0; i < 12; i++)
    {
        if (digitalRead(CONTROLLER_SNES_DATA) == LOW) snes_state |= (1 << i);
        digitalWrite(CONTROLLER_SNES_CLK, LOW);
        delayMicroseconds(6);
        digitalWrite(CONTROLLER_SNES_CLK, HIGH);
        delayMicroseconds(6);
    }

    // NES compatible bits
    state |= snes_state & 0xFF;

    // Map extra bits to A and B buttons
    if (snes_state & (1 << 8)) state |= (uint8_t)CONTROLLER::A;
    if (snes_state & (1 << 9)) state |= (uint8_t)CONTROLLER::B;
    if (snes_state & (1 << 10)) state |= (uint8_t)CONTROLLER::B;
    if (snes_state & (1 << 11)) state |= (uint8_t)CONTROLLER::A;

    return state;
}

static uint8_t SNESControllerRead()
{
    // --- 1. Tabla de mapeo de bits SNES a posiciones en 'snes_state' ---
    // Cada entrada indica a qué bit de snes_state debe ir el bit i del SNES.
    // Bits de salida (0-7): orden NES (A, B, Select, Start, Up, Down, Left, Right)
    // Bits 8-11: almacenan los botones que activan turbo (X, R, Y, L)
    static constexpr uint8_t map[12] = {
        1,   // SNES bit0 (B)     -> snes_state bit1 (emulador B)
        10,  // SNES bit1 (Y)     -> snes_state bit10 (turbo B, guardado)
        2,   // SNES bit2 (Select)-> snes_state bit2 (Select)
        3,   // SNES bit3 (Start) -> snes_state bit3 (Start)
        4,   // SNES bit4 (Up)    -> snes_state bit4 (Up)
        5,   // SNES bit5 (Down)  -> snes_state bit5 (Down)
        6,   // SNES bit6 (Left)  -> snes_state bit6 (Left)
        7,   // SNES bit7 (Right) -> snes_state bit7 (Right)
        0,   // SNES bit8 (A)     -> snes_state bit0 (emulador A)
        8,   // SNES bit9 (X)     -> snes_state bit8 (turbo A)
        11,  // SNES bit10 (L)    -> snes_state bit11 (turbo B)
        9    // SNES bit11 (R)    -> snes_state bit9 (turbo A)
    };

    // --- 2. Lectura y mapeo directo ---
    uint16_t snes_state = 0;
    digitalWrite(CONTROLLER_SNES_LATCH, HIGH);
    delayMicroseconds(12);
    digitalWrite(CONTROLLER_SNES_LATCH, LOW);
    delayMicroseconds(6);

    for (int i = 0; i < 12; i++)
    {
        if (digitalRead(CONTROLLER_SNES_DATA) == LOW)
            snes_state |= (1 << map[i]);   // Coloca el bit en la posición mapeada
        digitalWrite(CONTROLLER_SNES_CLK, LOW);
        delayMicroseconds(6);
        digitalWrite(CONTROLLER_SNES_CLK, HIGH);
        delayMicroseconds(6);
    }

    // --- 3. Estado base (bits 0-7 ya están en orden NES) ---
    uint8_t state = snes_state & 0xFF;

    // --- 4. Lógica de turbo (solo para los bits 0 y 1) ---
    // Botones que activan turbo A: bits 8 o 9 de snes_state (X o R)
    bool turboA = (snes_state & (1 << 8)) || (snes_state & (1 << 9));
    // Botones que activan turbo B: bits 10 o 11 (Y o L)
    bool turboB = (snes_state & (1 << 10)) || (snes_state & (1 << 11));

    // Variables estáticas para contadores (persisten entre llamadas)
    static uint8_t cntA = 0;
    static bool  turboA_state = false;
    static uint8_t cntB = 0;
    static bool  turboB_state = false;

    const uint8_t TURBO_FRAMES = 3;  // Ajusta la frecuencia (3 frames = ~20 Hz)

    // Turbo A
    if (turboA)
    {
        if (++cntA >= TURBO_FRAMES)
        {
            cntA = 0;
            turboA_state = !turboA_state;
        }
    }
    else
    {
        cntA = 0;
        turboA_state = false;
    }

    // Turbo B
    if (turboB)
    {
        if (++cntB >= TURBO_FRAMES)
        {
            cntB = 0;
            turboB_state = !turboB_state;
        }
    }
    else
    {
        cntB = 0;
        turboB_state = false;
    }

    // Aplicar turbo: el botón turbo tiene prioridad sobre el botón físico
    // Si el turbo está activo, forzamos el bit correspondiente (A = bit0, B = bit1)
    if (turboA_state)
        state |= (1 << 0);      // Activa A
    else if (turboA)
        state &= ~(1 << 0);     // Si hay turbo pero no está en el pulso, desactiva A

    if (turboB_state)
        state |= (1 << 1);      // Activa B
    else if (turboB)
        state &= ~(1 << 1);     // Desactiva B si turbo está activo pero en ciclo bajo

    return state;
}


static uint8_t PSXTransferByte(uint8_t byte)
{
    uint8_t temp = 0;
    for (int i = 0; i < 8; i++)
    {
        digitalWrite(CONTROLLER_PSX_COMMAND, (byte >> i) & 1);

        digitalWrite(CONTROLLER_PSX_CLK, LOW);

        digitalWrite(CONTROLLER_PSX_CLK, HIGH);
        delayMicroseconds(10);
        if (digitalRead(CONTROLLER_PSX_DATA) == LOW) temp |= (1 << i);
    }

    return temp;
}

static uint8_t PSXControllerRead()
{
    /*
    Communication Protocol
    First three bytes - Header
    Following bytes - Digital Mode (2 bytes) / Analog Mode (18 bytes)

    First byte
    Command: 0x01 (indicates new packet)
    Data: 0xFF

    Second byte
    Command: Main command (poll or configure controller)
             Polling: 0x42
    Data: Device Mode
          Upper 4 bits: mode (4 = digital, 7 = analog, F = config)
          Lower 4 bits: how many 16 bit words follow the header

    Third byte
    Command : 0x00
    Data: 0x5A
    */

    // Button Mappings
    /*
    Digital Mode
    0 - Select
    1 - L3
    2 - R3
    3 - Start
    4 - Up
    5 - Right
    6 - Down
    7 - Left
    8 - L2
    9 - R2
    10 - L1
    11 - R1
    12 - Triangle
    13 - O
    14 - X
    15 - Square

    Analog Mode
    - Analog sticks range 0x00 - 0xFF, 0x7F at rest
    - Pressure buttons range 0x00 - 0xFF, 0xFF is fully pressed
    0-15 - Same as digital mode
    Byte 2 - RX
    Byte 3 - RY
    Byte 4 - LX
    Byte 5 - LY
    Byte 6 - Right
    Byte 7 - Left
    Byte 8 - Up
    Byte 9 - Down
    Byte 10 - Triangle
    Byte 11 - O
    Byte 12 - X
    Byte 13 - Square
    Byte 14 - L1
    Byte 15 - R1
    Byte 16 - L2
    Byte 17 - R2
    */
    int b1, b2;
    uint8_t state = 0x00;
    uint16_t psx_state = 0x0000;

    // Initiate transfer
    delayMicroseconds(2);
    digitalWrite(CONTROLLER_PSX_ATTENTION, LOW);

    PSXTransferByte(0x01);
    PSXTransferByte(0x42);
    PSXTransferByte(0xFF);
    b1 = PSXTransferByte(0xFF);
    b2 = PSXTransferByte(0xFF);

    psx_state = (b2 << 8) | b1;

    // Map PSX bits to NES bits
    constexpr uint16_t PSX_SELECT = (1 << 0);
    constexpr uint16_t PSX_START = (1 << 3);
    constexpr uint16_t PSX_A_MASK = (1 << 11) | // R1
                                    (1 << 9) |  // R2
                                    (1 << 2) |  // R3
                                    (1 << 14) | // X
                                    (1 << 13);  // O
    constexpr uint16_t PSX_B_MASK = (1 << 10) | // L1
                                    (1 << 8) |  // L2
                                    (1 << 1) |  // L3
                                    (1 << 15) | // Square
                                    (1 << 12);  // Triangle
    constexpr uint16_t PSX_UP = (1 << 4);
    constexpr uint16_t PSX_DOWN = (1 << 6);
    constexpr uint16_t PSX_LEFT = (1 << 7);
    constexpr uint16_t PSX_RIGHT = (1 << 5);

    if (psx_state & PSX_SELECT) state |= (uint8_t)CONTROLLER::Select;
    if (psx_state & PSX_START) state |= (uint8_t)CONTROLLER::Start;

    if (psx_state & PSX_A_MASK) state |= (uint8_t)CONTROLLER::A;
    if (psx_state & PSX_B_MASK) state |= (uint8_t)CONTROLLER::B;

    if (psx_state & PSX_UP) state |= (uint8_t)CONTROLLER::Up;
    if (psx_state & PSX_DOWN) state |= (uint8_t)CONTROLLER::Down;
    if (psx_state & PSX_LEFT) state |= (uint8_t)CONTROLLER::Left;
    if (psx_state & PSX_RIGHT) state |= (uint8_t)CONTROLLER::Right;

    // End transfer
    digitalWrite(CONTROLLER_PSX_ATTENTION, HIGH);

    return state;
}

static uint8_t UartControllerRead()
{
    static uint8_t state = 0x00;
    static uint8_t no_data_count = 0;

    int b0 = Serial.read();
    int b1 = Serial1.read();
    if (b0 >= 0 || b1 >= 0)
    {
        // if received button presses from both Serial and Serial1 combine them
        state = 0x00;
        if (b0 >= 0) state = (uint8_t)b0;
        if (b1 >= 0) state |= (uint8_t)b1;

        no_data_count = 0;
        return state;
    }

    // if there is no data, then  reuse previous state 10 times before
    // setting state to 0x00 (no buttons pressed)
    no_data_count++;
    if (no_data_count >= 10)
    {
        state = 0x00;
        no_data_count = 10; // pin at 10 to prevent overflow
    }

    return state;
}

static uint8_t dummyControllerRead()
{
    return 0x00;
}

void initController(ControllerType controller_type)
{
    switch (controller_type)
    {
    case CT_GPIO:
        pinMode(A_BUTTON, INPUT_PULLUP);
        pinMode(B_BUTTON, INPUT_PULLUP);
        pinMode(LEFT_BUTTON, INPUT_PULLUP);
        pinMode(RIGHT_BUTTON, INPUT_PULLUP);
        pinMode(UP_BUTTON, INPUT_PULLUP);
        pinMode(DOWN_BUTTON, INPUT_PULLUP);
        pinMode(START_BUTTON, INPUT_PULLUP);
        pinMode(SELECT_BUTTON, INPUT_PULLUP);
        _controllerRead = gpioRead;
        break;

    case CT_NES:
        pinMode(CONTROLLER_NES_CLK, OUTPUT);
        pinMode(CONTROLLER_NES_LATCH, OUTPUT);
        pinMode(CONTROLLER_NES_DATA, INPUT);
        _controllerRead = NESControllerRead;
        break;

    case CT_SNES:
        pinMode(CONTROLLER_SNES_CLK, OUTPUT);
        pinMode(CONTROLLER_SNES_LATCH, OUTPUT);
        pinMode(CONTROLLER_SNES_DATA, INPUT);
        _controllerRead = SNESControllerRead;
        break;

    case CT_PSX:
        pinMode(CONTROLLER_PSX_DATA, INPUT_PULLUP);
        pinMode(CONTROLLER_PSX_COMMAND, OUTPUT);
        pinMode(CONTROLLER_PSX_ATTENTION, OUTPUT);
        pinMode(CONTROLLER_PSX_CLK, OUTPUT);

        digitalWrite(CONTROLLER_PSX_ATTENTION, HIGH);
        digitalWrite(CONTROLLER_PSX_CLK, HIGH);
        delayMicroseconds(10);

        // Dummy transfer bytes to clean internal controller state
        for (int i = 0; i < 2; i++)
        {
            digitalWrite(CONTROLLER_PSX_ATTENTION, LOW);
            delayMicroseconds(10);

            PSXTransferByte(0);
            delayMicroseconds(10);

            digitalWrite(CONTROLLER_PSX_ATTENTION, HIGH);
            delayMicroseconds(12);
        }
        _controllerRead = PSXControllerRead;
        break;
    case CT_UART:
#ifndef DEBUG
        // If DEBUG is defined then Serial.begin() was called during setup().
        // Here Serial is not being used for debugging but for reading button
        // states sent from a WebSerial game controller webpage over USB to serial.
        // debug messages will remain off
        Serial.begin(115200);
#endif

        // Serial1 is used by an adapter board that supports multiple controller types and
        // sends the button presses over serial. A different serial port is used instead of
        // Serial (Serial0) to prevent the adapter from interfering with programming.
        // Arduino ignores parity errors so there is nothing to be gained by setting the parity bit
        Serial1.begin(115200, SERIAL_8N1, CONTROLLER_UART_RX, CONTROLLER_UART_TX);
        delay(200); // allow controller adapter to finish booting

        Serial1.write(hw_config.controller_type);
        _controllerRead = UartControllerRead;
        break;
    case CT_NC:
    default: _controllerRead = dummyControllerRead; break;
    }
}
