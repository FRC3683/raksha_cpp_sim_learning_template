#pragma once

// basically a 2-state FSM
struct schedulable {
    bool entered = false;
    bool scheduled = false;

    virtual void enter();
    virtual void periodic();
    virtual void exit();

public:
    void schedule();
    void process();
    void cancel();
    bool is_scheduled();
};
