#include "dave/schedulable.hpp"

void schedulable::enter() {}
void schedulable::periodic() {}
void schedulable::exit() {}

void schedulable::schedule() {
    if( scheduled ) {
        return;
    }
    entered = false;
    scheduled = true;
}

void schedulable::cancel() {
    if( !scheduled ) {
        return;
    }
    scheduled = false;
    exit();
}

bool schedulable::is_scheduled() {
    return scheduled;
}

void schedulable::process() {
    if( !scheduled ) {
        return;
    }
    if( !entered ) {
        enter();
        entered = true;
    }
    periodic();
}
