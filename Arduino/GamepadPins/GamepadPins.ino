/*
 *  Project     Arduino XInput Library
 *  @author     David Madison
 *  @link       github.com/dmadison/ArduinoXInput
 *  @license    MIT - Copyright (c) 2019 David Madison
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 *  Example:      GamepadPins
 *  Description:  Uses all of the available pin inputs to build a 'complete'
 *                Xbox gamepad, with both analog joysticks, both triggers,
 *                and all of the main buttons.
 *
 *                * Joysticks should be your typical 10k dual potentiometers.
 *                  To prevent random values caused by floating inputs,
                    joysticks are disabled by default.
 *                * Triggers can be either analog (pots) or digital (buttons).
 *                  Set the 'TriggerButtons' variable to change between the two.
 *                * Buttons use the internal pull-ups and should be connected
 *                  directly to ground.
 *
 *                These pins are designed around the Leonardo's layout. You
 *                may need to change the pin numbers if you're using a
 *                different board type
 *
 */

#include <XInput.h>

// Setup

// Button Pins
const int Pin_Button_01 = 1;
const int Pin_Button_02 = 2;
const int Pin_Button_03 = 3;
const int Pin_Button_04 = 4;
const int Pin_Button_05 = 5;
const int Pin_Button_06 = 6;
const int Pin_Button_07 = 7;
const int Pin_Button_08 = 8;
const int Pin_Button_09 = 9;
const int Pin_Button_10 = 10;
const int Pin_Button_11 = 11;
const int Pin_Button_12 = 12;


void setup() {

	// Set buttons as inputs, using internal pull-up resistors
	pinMode(Pin_Button_01, INPUT_PULLUP);
  pinMode(Pin_Button_02, INPUT_PULLUP);
  pinMode(Pin_Button_03, INPUT_PULLUP);
  pinMode(Pin_Button_04, INPUT_PULLUP);
  pinMode(Pin_Button_05, INPUT_PULLUP);
  pinMode(Pin_Button_06, INPUT_PULLUP);
  pinMode(Pin_Button_07, INPUT_PULLUP);
  pinMode(Pin_Button_08, INPUT_PULLUP);
  pinMode(Pin_Button_09, INPUT_PULLUP);
  pinMode(Pin_Button_10, INPUT_PULLUP);
  pinMode(Pin_Button_11, INPUT_PULLUP);
  pinMode(Pin_Button_12, INPUT_PULLUP);

	XInput.setAutoSend(false);  // Wait for all controls before sending

	XInput.begin();
}

void loop() {
	// Read pin values and store in variables
	// (Note the "!" to invert the state, because LOW = pressed)
	boolean button_01 = !digitalRead(Pin_Button_01);
	boolean button_02 = !digitalRead(Pin_Button_02);
	boolean button_03 = !digitalRead(Pin_Button_03);
	boolean button_04 = !digitalRead(Pin_Button_04);
	boolean button_05 = !digitalRead(Pin_Button_05);
	boolean button_06 = !digitalRead(Pin_Button_06);
	boolean button_07 = !digitalRead(Pin_Button_07);
	boolean button_08 = !digitalRead(Pin_Button_08);
	boolean button_09 = !digitalRead(Pin_Button_09);
	boolean button_10 = !digitalRead(Pin_Button_10);
	boolean button_11 = !digitalRead(Pin_Button_11);
	boolean button_12 = !digitalRead(Pin_Button_12);
	
	// Set XInput buttons
	XInput.setButton(BUTTON_A, button_01);
	XInput.setButton(BUTTON_B, button_02);
	XInput.setButton(BUTTON_X, button_03);
	XInput.setButton(BUTTON_Y, button_04);

	XInput.setButton(BUTTON_LB, button_05);
	XInput.setButton(BUTTON_RB, button_06);

	XInput.setButton(BUTTON_L3, button_07);
	XInput.setButton(BUTTON_R3, button_08);

	// Set XInput DPAD values
	XInput.setDpad(button_09, button_10, button_11, button_12);

	// Send control data to the computer
	XInput.send();
}
