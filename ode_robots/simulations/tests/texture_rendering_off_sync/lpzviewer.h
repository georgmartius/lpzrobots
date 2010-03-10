
#include <osgViewer/Viewer>
#include <osg/Group>


/** Viewer holds a single view on to a single scene 
    that supports the rendering of offscreen RRT (render to texture) cameras
    at any time (without sync)
*/
class LPZViewer : public osgViewer::Viewer
{
public:
  
  LPZViewer();

  LPZViewer(osg::ArgumentParser& arguments);

  LPZViewer(const osgViewer::Viewer& viewer, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY);

  virtual ~LPZViewer();

  virtual void renderOffScreen();

  void addOffScreenRRTNode(osg::Node* node);
  void removeOffScreenRRTNode(osg::Node* node);

protected:
  virtual void offScreenRenderingTraversals();
  osg::ref_ptr<osg::Group> offScreenNodes;

  void lpzviewerConstructorInit();

};
