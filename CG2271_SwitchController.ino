/*
 *  Mapping of Switch Pro Controller buttons:
 *
 *  ********** Motor Controls **********
 *  A : Forward
 *  B : Reverse
 *  X : Fast Reverse
 *  Y : Fast Forward
 *  L : Forward Left
 *  R : Forward Right
 *  ZL : Fast Forward Left
 *  ZR : Fast Forward Right
 *  Thumbstick Left : Fast Reverse Left
 *  Thumbstick Right : Fast Reverse Right
 *  Home : Stop
 *
 *  ********** Song Controls **********
 *  - : On normal song
 *  + : On alternate song
 */

#define INBUILT_LED 2

#define FORWARD 192
#define FAST_FORWARD 224

#define REVERSE 32
#define FAST_REVERSE 0

#define FORWARD_LEFT 119
#define FAST_FORWARD_LEFT 134

#define FORWARD_RIGHT 217
#define FAST_FORWARD_RIGHT 218

#define LEFT 56
#define FAST_LEFT 28
#define RIGHT 168
#define FAST_RIGHT 196
#define STOP 112

#define REVERSE_LEFT 105
#define FAST_REVERSE_LEFT 90
#define REVERSE_RIGHT 7
#define FAST_REVERSE_RIGHT 6

#define NORMAL_SONG 225
#define ALTERNATE_SONG 226

#include <Arduino.h>
#include <Bluepad32.h>
#include <vector>

using namespace std;

ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// To store commands to be sent to KL25Z
// vector<byte> commands;
// commands.push_back(STOP);
byte curr_command = STOP;
byte prev_command = STOP; // Init with random prev command

void checkCommand()
{
  // byte curr_command = commands[commands.size()];
  if (curr_command != prev_command)
  {
    Serial.write(curr_command);
  }

  prev_command = curr_command;
}

// Arduino setup function. Runs in CPU 1
void setup()
{
  // Initialize serial
  Serial.begin(9600, SERIAL_8N1);
  pinMode(INBUILT_LED, OUTPUT);
  while (!Serial)
  {
    // wait for serial port to connect.
    // You don't have to do this in your game. This is only for debugging
    // purposes, so that you can see the output in the serial console.
    ;
  }

  // This call is mandatory. It setups Bluepad32 and creates the callbacks.
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But might also fix some connection / re-connection issues.
  //  BP32.forgetBluetoothKeys();
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl)
{
  // To indicate controller connected
  digitalWrite(INBUILT_LED, HIGH);
  if (myControllers[0] == nullptr)
  {
    myControllers[0] = ctl;
  }
  // Default example code
  //  bool foundEmptySlot = false;
  //  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
  //    if (myControllers[i] == nullptr) {
  //      Serial.print("CALLBACK: Controller is connected, index=");
  //      Serial.println(i);
  //      myControllers[i] = ctl;
  //      foundEmptySlot = true;
  //
  //      // Optional, once the gamepad is connected, request further info about the
  //      // gamepad.
  //      ControllerProperties properties = ctl->getProperties();
  //      char buf[80];
  //      sprintf(buf,
  //              "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
  //              "flags: 0x%02x",
  //              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
  //              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
  //              properties.vendor_id, properties.product_id, properties.flags);
  //      Serial.println(buf);
  //      break;
  //    }
  //  }
  //  if (!foundEmptySlot) {
  //    Serial.println(
  //        "CALLBACK: Controller connected, but could not found empty slot");
  //  }
}

void onDisconnectedController(ControllerPtr ctl)
{
  bool foundGamepad = false;
  for (int i = 0; i < 1; i++)
  {
    if (myControllers[i] == ctl)
    {
      //      Serial.print("CALLBACK: Controller is disconnected from index=");
      //      Serial.println(i);
      // To indicate controller disconnected
      digitalWrite(INBUILT_LED, LOW);
      myControllers[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }
  //  if (!foundGamepad) {
  //    Serial.println(
  //        "CALLBACK: Controller disconnected, but not found in myControllers");
  //  }
}

void processGamepad(ControllerPtr gamepad)
{
  // Break out early if no buttons are pressed
  if (gamepad->buttons() == 0x0000 && gamepad->dpad() == 0x00 && gamepad->miscButtons() == 0x00)
  {
    if (curr_command != STOP)
    {
      curr_command = STOP;
    }
    return;
  }

  // Check DPAD
  if (gamepad->dpad() == 0x04)
  {
    curr_command = RIGHT;
    return;
  }
  else if (gamepad->dpad() == 0x08)
  {
    curr_command = LEFT;
    return;
  }

  // Check buttons
  if ((gamepad->buttons() & 0x0010) == 0x0010)
  {
    curr_command = FORWARD_LEFT;
    return;
  }
  else if ((gamepad->buttons() & 0x0020) == 0x0020)
  {
    curr_command = FORWARD_RIGHT;
    return;
  }
  else if ((gamepad->buttons() & 0x0040) == 0x0040)
  {
    curr_command = FAST_FORWARD_LEFT;
    return;
  }
  else if ((gamepad->buttons() & 0x0080) == 0x0080)
  {
    curr_command = FAST_FORWARD_RIGHT;
    return;
  }
  else if ((gamepad->buttons() & 0x0100) == 0x0100)
  {
    curr_command = FAST_REVERSE_LEFT;
    return;
  }
  else if ((gamepad->buttons() & 0x0200) == 0x0200)
  {
    curr_command = FAST_REVERSE_RIGHT;
    return;
  }

  // Need to put forward and reverse at the ends
  if ((gamepad->buttons() & 0x0001) == 0x0001)
  {
    curr_command = REVERSE;
    return;
  }
  else if ((gamepad->buttons() & 0x0002) == 0x0002)
  {
    curr_command = FORWARD;
    return;
  }
  else if ((gamepad->buttons() & 0x0004) == 0x0004)
  {
    curr_command = FAST_FORWARD;
    return;
  }
  else if ((gamepad->buttons() & 0x0008) == 0x0008)
  {
    curr_command = FAST_REVERSE;
    return;
  }

  // Song + STOP buttons
  if (gamepad->miscButtons() == 0x01)
  {
    curr_command = STOP;
    return;
  }
  if (gamepad->miscButtons() == 0x02)
  {
    curr_command = NORMAL_SONG;
    return;
  }
  if (gamepad->miscButtons() == 0x04)
  {
    curr_command = ALTERNATE_SONG;
    return;
  }

  // Another way to query the buttons, is by calling buttons(), or
  // miscButtons() which return a bitmask.
  // Some gamepads also have DPAD, axis and more.
  //  char buf[256];
  //  snprintf(buf, sizeof(buf) - 1,
  //           "idx=%d, dpad: 0x%02x, buttons: 0x%04x, "
  //           "axis L: %4li, %4li, axis R: %4li, %4li, "
  //           "brake: %4ld, throttle: %4li, misc: 0x%02x, "
  //           "gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d, "
  //           "battery: %d",
  //           gamepad->index(),        // Gamepad Index
  //           gamepad->dpad(),         // DPAD
  //           gamepad->buttons(),      // bitmask of pressed buttons
  //           gamepad->axisX(),        // (-511 - 512) left X Axis
  //           gamepad->axisY(),        // (-511 - 512) left Y axis
  //           gamepad->axisRX(),       // (-511 - 512) right X axis
  //           gamepad->axisRY(),       // (-511 - 512) right Y axis
  //           gamepad->brake(),        // (0 - 1023): brake button
  //           gamepad->throttle(),     // (0 - 1023): throttle (AKA gas) button
  //           gamepad->miscButtons(),  // bitmak of pressed "misc" buttons
  //           gamepad->gyroX(),      // Gyro X
  //           gamepad->gyroY(),      // Gyro Y
  //           gamepad->gyroZ(),      // Gyro Z
  //           gamepad->accelX(),     // Accelerometer X
  //           gamepad->accelY(),     // Accelerometer Y
  //           gamepad->accelZ(),     // Accelerometer Z
  //           gamepad->battery()       // 0=Unknown, 1=empty, 255=full
  //  );
  //  Serial.println(buf);

  // You can query the axis and other properties as well. See
  // Controller.h For all the available functions.
}

// Arduino loop function. Runs in CPU 1
void loop()
{
  // This call fetches all the controller info from the NINA (ESP32) module.
  // Just call this function in your main loop.
  // The controllers pointer (the ones received in the callbacks) gets updated
  // automatically.
  BP32.update();

  // It is safe to always do this before using the controller API.
  // This guarantees that the controller is valid and connected.
  ControllerPtr myController = myControllers[0];

  if (myController && myController->isConnected())
  {
    processGamepad(myController);
    checkCommand();
  }
  // delay(1);
}
