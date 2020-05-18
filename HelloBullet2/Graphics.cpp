#include "Graphics.h"
#include <iostream>

// Implementation of constructor
Graphics::Graphics(IrrlichtDevice* device) : m_device(device) {
    m_driver = m_device->getVideoDriver();
    m_smgr = m_device->getSceneManager();
    m_gui = m_device->getGUIEnvironment();
}

// Implementation of Graphics::drawLine
void Graphics::drawLine(const vector3df& from, const vector3df& to, const SColorf& fromColor, const SColorf& tocolor)
{
    matrix4 id;
    id.makeIdentity();
    m_driver->setTransform(video::ETS_WORLD, id);
    m_driver->draw3DLine(from, to, fromColor.toSColor());
}

IVideoDriver* Graphics::getVideoDriver() {
    return this->m_driver;
}

ISceneManager* Graphics::getSceneManager() {
    return this->m_smgr;
}

IGUIEnvironment* Graphics::getGUIEnvironment() {
    return m_gui;
}
