#include "MyEventReceiver.h"

bool MyEventReceiver::OnEvent(const SEvent& event)
{
    // Remember the mouse state
    if (event.EventType == irr::EET_MOUSE_INPUT_EVENT)
    {
        switch (event.MouseInput.Event)
        {
        case EMIE_LMOUSE_PRESSED_DOWN:
            MouseState.LeftButtonDown = true;
            break;

        case EMIE_LMOUSE_LEFT_UP:
            MouseState.LeftButtonDown = false;
            break;

        case EMIE_MOUSE_MOVED:
            MouseState.Position.X = event.MouseInput.X;
            MouseState.Position.Y = event.MouseInput.Y;
            break;

        default:
            // We won't use the wheel
            break;
        }
    }

    if (event.EventType == irr::EET_KEY_INPUT_EVENT)
        KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;

    // The state of each connected joystick is sent to us
    // once every run() of the Irrlicht device.  Store the
    // state of the first joystick, ignoring other joysticks.
    // This is currently only supported on Windows and Linux.
    if (event.EventType == irr::EET_JOYSTICK_INPUT_EVENT
        && event.JoystickEvent.Joystick == 0)
    {
        JoystickState = event.JoystickEvent;
    }

    return false;
}

const SEvent::SJoystickEvent& MyEventReceiver::GetJoystickState(void) const
{
    return JoystickState;
}

const MyEventReceiver::SMouseState& MyEventReceiver::GetMouseState(void) const
{
    return MouseState;
}

bool MyEventReceiver::IsKeyDown(EKEY_CODE keyCode) const
{
    return KeyIsDown[keyCode];
}

MyEventReceiver::MyEventReceiver()
{
    for (u32 i = 0; i < KEY_KEY_CODES_COUNT; ++i)
        KeyIsDown[i] = false;
}
