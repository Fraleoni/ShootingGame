#pragma once
//#include "D:\Progetti\vcpkg\installed\x86-windows\include\bullet\LinearMath\btIDebugDraw.h"
//#include "btBulletDynamicsCommon.h"
#include "irrlicht.h"
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
using namespace std;
class Graphics
{
public:
	// Costruttore 
	IrrlichtDevice* m_device = 0;
	IVideoDriver* m_driver = 0;
	ISceneManager* m_smgr = 0;
	IGUIEnvironment* m_gui = 0;
	Graphics(IrrlichtDevice* device);
	void drawLine(const vector3df& from, const vector3df& to, const SColorf& fromColor, const SColorf& tocolor);
	IVideoDriver* getVideoDriver();
	ISceneManager* getSceneManager();
	IGUIEnvironment* getGUIEnvironment();

};

