#ifndef _JOY_MSG_HPP
#define _JOY_MSG_HPP

#include <stdint.h>

#define BUTTONS(button) (1 << button)

namespace PS4
{
  enum Buttons
  {
    CROSS,
    CIRCLE,
    SQUARE,
    TRIANGLE,
    SHARE,
    PS,
    OPTION,
    L3,
    R3,
    L1,
    R1,
    UP,
    DOWN,
    LEFT,
    RIGHT,
    TOUCH
  };
}

namespace XBox
{
  enum Buttons
  {
    A,
    B,
    X,
    Y,
    BACK,
    XBOX,
    START,
    LSTICL,
    RSTICK,
    LB,
    RB,
    UP,
    DOWN,
    LEFT,
    RIGHT
  };
}

typedef struct
{
  uint8_t battery : 4; // Battery level (0-15)
  uint8_t usb : 1;     // USB connection status (1 if connected)
  uint8_t audio : 1;   // Audio jack connection status (1 if connected)
  uint8_t mic : 1;     // Microphone connection status (1 if connected)
  uint8_t unknown : 1; // Possibly for the extension port
} Status __attribute__((packed));

#pragma pack(push, 1)
struct JoyData
{
  int8_t  lx;           // Left stick X
  int8_t  ly;           // Left stick Y
  int8_t  rx;           // Right stick X
  int8_t  ry;           // Right stick Y
  uint8_t lt;           // Left trigger
  uint8_t rt;           // Right trigger
  uint16_t buttons;     // Button bitmask
  Status status;        // 1 byte: battery + flags
  int16_t accX;         // Accelerometer X
  int16_t accZ;         // Accelerometer Z
  int16_t accY;         // Accelerometer Y
};
#pragma pack(pop)

#endif // _JOY_MSG_HPP