#pragma once
#include <irrlicht.h>
using namespace irr;

class MyShaderCallBack :
	public video::IShaderConstantSetCallBack
{
public:
	MyShaderCallBack(IrrlichtDevice* pDevice, bool useHighLevelShaders, bool useCgShaders);
	virtual void OnSetConstants(video::IMaterialRendererServices* services,
		s32 userData);
};

