/*
 * Linear slider controller
 * Author: Luke Strohbehn
 * April 10, 2023
 * 
 * 
 * The structure of this script is based on two normally-closed limit switches, configured 
 * with the onboard input resistors using INPUT_PULLUP. Adjust the logic according to your
 * hardware logic in limitswitch.cpp
 * 
 * 
 * 
 */
#ifndef DEBUG
#define DEBUG 0
#endif

#include "limitswitch.h"
#include "EthTCP.h"
#include <AccelStepper.h>
#include <EEPROM.h>
#include <stdio.h>
 
// Limit switch pins
const uint8_t limit_switch0 {2};
const uint8_t limit_switch1 {3};

// Stepper pins
const uint8_t spin0 {4};
const uint8_t spin1 {5};

// EEPROM position address
int EEPROM_pos_addr {0};
int EEPROM_write_counter;
int EEPROM_write_counter_addr;

// Desired stepper position
float stepper_target;

// Switches
LimitSwitch ls0(limit_switch0);
LimitSwitch ls1(limit_switch1);

// Stepper
AccelStepper stepper(1, spin0, spin1);

// Ethernet
Eth eth0;
bool local_client_connected = false;


void update_stepper(int pos) {
    /* Increment stepper position for event loop */
    stepper.moveTo(pos); // moveTo will set target to an absolute position, runs at the last set speed
    if (abs(stepper.distanceToGo()) > 0) {
        stepper.run();
        Serial.println(stepper.currentPosition());
    } else {
        stepper.disableOutputs();
        // Serial.println(F("Done."));
        // EEPROM write takes 3.3ms, can only be written 100,000 times. Only use for when position finishes updating? using `update()` only writes if the value is different
        // EEPROM.update(EEPROM_pos_addr, stepper.currentPosition)
    }
}

void calibrate_stepper() {
    /* Zeros stepper position to limit_switch0 */

}

void setup_stepper() {
    // Stepper setup
    stepper.setMaxSpeed(5000);
    stepper.setAcceleration(1000);
    stepper.disableOutputs(); // re-enable pins with enableOutputs();
    stepper.setCurrentPosition(0); // to be used in calibration step. Store last position in eeprom?
}


/*******************************************

    Main

********************************************/

void setup() {
#if DEBUG
    Serial.begin(115200);
    while (!Serial){;}
#endif // DEBUG

    // Ethernet connections
    eth0.begin_ethernet();
    eth0.begin_server();
    eth0.connect_local_client();

    setup_stepper();
}



void loop() {
    /* Server */
    // Remove any stale clients, add new ones
    eth0.remove_clients();
    eth0.accept_clients();
    // Read data collected by the server
    eth0.read_data(); // new data stored in Eth::receivedChars

    /* Client */
    // Reconnect local_client to lan server if broken, and maintain DHCP lease
    local_client_connected = eth0.connect_local_client();

    // Send position data from the local client
    if (local_client_connected) {
        eth0.send_data(stepper.currentPosition()); // update to stepper position/velocity later
    }

    
    /* Update stepper target or run calibration if newData */
    if (eth0.newData) {
#if DEBUG
        Serial.print(F("Received command: "));
        Serial.println(eth0.receivedChars);
#endif // DEBUG

        stepper_target = atol(eth0.receivedChars); // might need to change to atol if sending velocities?
#if DEBUG
        Serial.print(F("stepper_target: "));
        Serial.println(stepper_target, 5);
#endif // DEBUG

        eth0.newData = false;
    }
    

    /* Interrupts */
    // NOTE*********** does not stop stepper in loop, need to set another flag
    if (ls0.read_switch()) {
#if DEBUG
        Serial.println(F("Interrupt 0 triggered"));
#endif // DEBUG
        stepper.stop();
        stepper.runToPosition();
        stepper_target = stepper.currentPosition();
    }
    if (ls1.read_switch()) {
#if DEBUG
        Serial.println(F("Interrupt 1 triggered"));
#endif // DEBUG
        stepper.stop();
        stepper.runToPosition();
        stepper_target = stepper.currentPosition();
    }

    /* Stepper */
    update_stepper(stepper_target); // Custom stepper class, holds target position and always tries to move there?
    
}
