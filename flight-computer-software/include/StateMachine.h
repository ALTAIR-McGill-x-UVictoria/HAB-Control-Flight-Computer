#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>

template <int MAX_STATES, int MAX_TRANSITIONS>
class StateMachine {
public:
    using StateFunc = void (*)();
    using TransitionFunc = bool (*)();

    /*
        * State structure
        * @param id: State identifier
        * @param onEnter: Function to be called when entering the state
        * @param onExecute: Function to be called when in the state
        * @param onExit: Function to be called when exiting the state
    */
    struct State {
        int id;
        StateFunc onEnter;
        StateFunc onExecute;
        StateFunc onExit;
    };

    /*
        * Transition structure
        * @param fromState: State to transition from
        * @param toState: State to transition to
        * @param condition: Function that returns a boolean to determine if the transition should occur
    */
    struct Transition {
        int fromState;
        int toState;
        TransitionFunc condition;
    };

    /*
        * Add a state to the state machine
        * @param id: State identifier
        * @param onEnter: Function to be called when entering the state
        * @param onExecute: Function to be called when in the state
        * @param onExit: Function to be called when exiting the state
        * @return void
    */
    void addState(int id, StateFunc onEnter = nullptr, StateFunc onExecute = nullptr, StateFunc onExit = nullptr) {
        if (stateCount < MAX_STATES) {
            states[stateCount++] = {id, onEnter, onExecute, onExit};
        }
    }

    /*
        * Add a transition to the state machine
        * @param fromState: State to transition from
        * @param toState: State to transition to
        * @param condition: Function that returns a boolean to determine if the transition should occur
        * @return void
    */
    void addTransition(int fromState, int toState, TransitionFunc condition) {
        if (transitionCount < MAX_TRANSITIONS) {
            transitions[transitionCount++] = {fromState, toState, condition};
        }
    }

    /*
        * Set the initial state of the state machine
        * @param id: State identifier
        * @return void
    */
    void setInitialState(int id) {
        currentState = findStateIndex(id);
        if (currentState != -1 && states[currentState].onEnter) {
            states[currentState].onEnter();
        }
    }

    /*
        * Update the state machine
        * @return void
    */
    void update() {
        // Check transitions
        for (int i = 0; i < transitionCount; i++) {
            if (transitions[i].fromState == states[currentState].id && transitions[i].condition()) {
                changeState(transitions[i].toState);
                break;
            }
        }

        // Execute current state function
        if (currentState != -1 && states[currentState].onExecute) {
            states[currentState].onExecute();
        }
    }

private:
    State states[MAX_STATES];
    Transition transitions[MAX_TRANSITIONS];
    int stateCount = 0;
    int transitionCount = 0;
    int currentState = -1;

    int findStateIndex(int id) {
        for (int i = 0; i < stateCount; i++) {
            if (states[i].id == id) return i;
        }
        return -1;
    }

    void changeState(int newState) {
        int newIndex = findStateIndex(newState);
        if (newIndex == -1) return;

        if (states[currentState].onExit) states[currentState].onExit();
        currentState = newIndex;
        if (states[currentState].onEnter) states[currentState].onEnter();
    }
};

#endif // STATE_MACHINE_H