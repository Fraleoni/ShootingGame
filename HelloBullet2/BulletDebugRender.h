#pragma once
#include "D:\Progetti\vcpkg\installed\x86-windows\include\bullet\LinearMath\btIDebugDraw.h"
#include "btBulletDynamicsCommon.h"
#include "irrlicht.h"
#include "Graphics.h"
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
using namespace std;

class BulletDebugRender :
    public btIDebugDraw
{
public:
    BulletDebugRender();
    BulletDebugRender(Graphics* g);
    virtual ~BulletDebugRender();
    virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color);
    virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& fromColor, const btVector3& toColor);

    virtual void    drawSphere(const btVector3& p, btScalar radius, const btVector3& color);
    virtual void    drawTriangle(const btVector3& a, const btVector3& b, const btVector3& c, const btVector3& color, btScalar alpha);
    virtual void    drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color);
    virtual void    reportErrorWarning(const char* warningString);
    virtual void    draw3dText(const btVector3& location, const char* textString);
    virtual void    setDebugMode(int debugMode);
    virtual int     getDebugMode() const { return m_debugMode; }

    void setGraphics(Graphics* g);
private:
    Graphics* m_graphics;
    int m_debugMode = DBG_DrawWireframe;
};




