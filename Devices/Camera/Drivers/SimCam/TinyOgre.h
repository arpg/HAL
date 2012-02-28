/*
 * File:   TinyOgre.h
 * Author: jmf
 *
 * Created on February 22, 2012, 11:26 AM
 */

#ifndef TINYOGRE_H
#define	TINYOGRE_H

#include <OgreRoot.h>
#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreLogManager.h>
#include <OgreViewport.h>
#include <OgreConfigFile.h>
#include <OgreEntity.h>
#include <OgreWindowEventUtilities.h>


class TinyOgre
{
public:
    TinyOgre(void);
    virtual ~TinyOgre(void);
    bool go(void);
protected:
    Ogre::Root *mRoot;
    Ogre::Camera* mCamera;
    Ogre::SceneManager* mSceneMgr;
    Ogre::RenderWindow* mWindow;
    Ogre::String mResourcesCfg;
    Ogre::String mPluginsCfg;
};



//-------------------------------------------------------------------------------------
TinyOgre::TinyOgre(void)
    : mRoot(0),
    mCamera(0),
    mSceneMgr(0),
    mWindow(0),
    mResourcesCfg(Ogre::StringUtil::BLANK),
    mPluginsCfg(Ogre::StringUtil::BLANK)
{
}
//-------------------------------------------------------------------------------------
TinyOgre::~TinyOgre(void)
{
    delete mRoot;
}

bool TinyOgre::go(void)
{
#ifdef _DEBUG
    mResourcesCfg = "resources_d.cfg";
    mPluginsCfg = "plugins_d.cfg";
#else
    mResourcesCfg = "resources.cfg";
    mPluginsCfg = "plugins.cfg";
#endif

    // construct Ogre::Root
    mRoot = new Ogre::Root(mPluginsCfg);

//-------------------------------------------------------------------------------------
    // setup resources
    // Load resource paths from config file
    Ogre::ConfigFile cf;
    cf.load(mResourcesCfg);

    // Go through all sections & settings in the file
    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

    Ogre::String secName, typeName, archName;
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                archName, typeName, secName);
        }
    }
//-------------------------------------------------------------------------------------
    // configure
    // Show the configuration dialog and initialise the system
    // You can skip this and use root.restoreConfig() to load configuration
    // settings if you were sure there are valid ones saved in ogre.cfg
    if(mRoot->restoreConfig() || mRoot->showConfigDialog())
    {
        // If returned true, user clicked OK so initialise
        // Here we choose to let the system create a default rendering window by passing 'true'
        mWindow = mRoot->initialise(true, "TinyOgre Render Window");
    }
    else
    {
        return false;
    }
//-------------------------------------------------------------------------------------
    // choose scenemanager
    // Get the SceneManager, in this case a generic one
    mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
//-------------------------------------------------------------------------------------
	/// RPG: USER HAS TO SPECIFY THIS INITIAL CAM POSITION


    // create camera
    // Create the camera
    mCamera = mSceneMgr->createCamera("PlayerCam");

    // Position it at 500 in Z direction
    mCamera->setPosition(Ogre::Vector3(0,0,80));
    // Look back along -Z
    mCamera->lookAt(Ogre::Vector3(0,0,-300));
    mCamera->setNearClipDistance(5);

//-------------------------------------------------------------------------------------
    // create viewports
    // Create one viewport, entire window
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(0,0,0));

    // Alter the camera aspect ratio to match the viewport
    mCamera->setAspectRatio(
        Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
//-------------------------------------------------------------------------------------
    // Set default mipmap level (NB some APIs ignore this)
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
//-------------------------------------------------------------------------------------
    // Create any resource listeners (for loading screens)
    //createResourceListener();
//-------------------------------------------------------------------------------------
    // load resources
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
//-------------------------------------------------------------------------------------
    // Create the scene
    Ogre::Entity* ogreHead = mSceneMgr->createEntity("Head", "ogrehead.mesh");

    Ogre::SceneNode* headNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    headNode->attachObject(ogreHead);

    // Set ambient light
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

    // Create a light
    Ogre::Light* l = mSceneMgr->createLight("MainLight");
    l->setPosition(20,80,50);
//-------------------------------------------------------------------------------------
/*
//Added code from Int Tut 7 -------------------------------------------------
// 1  create texture
Ogre::TexturePtr rtt_texture = Ogre::TextureManager::getSingleton().createManual("RttTex", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, mWindow->getWidth(), mWindow->getHeight(), 0, Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);

// 2  get render target and set params
Ogre::RenderTexture *renderTexture = rtt_texture->getBuffer()->getRenderTarget();

renderTexture->addViewport(mCamera);
renderTexture->getViewport(0)->setClearEveryFrame(true);
renderTexture->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
renderTexture->getViewport(0)->setOverlaysEnabled(false);

// 3  update render texture and save to file
// Either this way
//renderTexture->setAutoUpdated(true);
// or this way
renderTexture->update();

// Now save the contents
renderTexture->writeContentsToFile("start.png");
*/
//-------------------------------------------------------------------------------------

    while(true)
    {
        // Pump window messages for nice behaviour
        Ogre::WindowEventUtilities::messagePump();

        if(mWindow->isClosed())
        {
            return false;
        }

        // Render a frame
        if(!mRoot->renderOneFrame()) return false;
    }

    // We should never be able to reach this corner
    // but return true to calm down our compiler
    return true;
}

#endif	/* TINYOGRE_H */

