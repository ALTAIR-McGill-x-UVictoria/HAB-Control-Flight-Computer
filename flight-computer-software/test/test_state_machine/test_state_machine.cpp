#include <Arduino.h>
#include "../../include/StateMachine.h"

enum State {
    STATE1,
    STATE2,
    STATE3
};

void onState1();
void onEnterState2();
void onExitState2();
void onState3();
void onExitState3();

StateMachine<3, 3> fsm; // 3 states, 3 transitions
bool timeout = false;

void setup() {
    fsm.addState(STATE1, nullptr, onState1, nullptr);
    fsm.addState(STATE2, onEnterState2, nullptr, onExitState2);
    fsm.addState(STATE3, nullptr, onState3, onExitState3);

    fsm.addTransition(STATE1, STATE2, []() { return true; });
    fsm.addTransition(STATE2, STATE3, []() { return true; });
    fsm.addTransition(STATE3, STATE1, []() { return timeout; });

    fsm.setInitialState(STATE1);
}

void loop () {
    fsm.update();

    static unsigned long timer = millis();
    if (millis() - timer > 3000) {
        timeout = true;
    }

    delay(100);
}

void onState1() {
    Serial.println("In state 1");
}

void onEnterState2() {
    Serial.println("Entering state 2");
}

void onExitState2() {
    Serial.println("Exiting state 2");
}

void onState3() {
    Serial.println("In state 3");
}

void onExitState3() {
    timeout = false;
    Serial.println("Exiting state 3");
}