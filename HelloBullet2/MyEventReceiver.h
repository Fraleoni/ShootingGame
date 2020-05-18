#pragma once
#include <irrlicht.h>

using namespace irr; 
class MyEventReceiver :
	public IEventReceiver
{
public:
    struct SMouseState
    {
        core::position2di Position;
        bool LeftButtonDown;
        SMouseState() : LeftButtonDown(false) { }
    } MouseState;

    MyEventReceiver();
    const SEvent::SJoystickEvent& GetJoystickState(void) const;
    const SMouseState& GetMouseState(void) const;
    bool IsKeyDown(EKEY_CODE keyCode) const;
    virtual bool OnEvent(const SEvent& event);

private:
    SEvent::SJoystickEvent JoystickState;
    // We use this array to store the current state of each key
    bool KeyIsDown[KEY_KEY_CODES_COUNT];
    bool isKeyRead;
};

