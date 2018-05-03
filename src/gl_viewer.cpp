#include "gl_viewer.h"
#include "ros/ros.h"
#include <QtGui>
#include <QtOpenGL>
#include <QThread>
#include <GL/glut.h>
#include "boost/foreach.hpp"
#include <cmath>
#include <QApplication>
#include <QAction>
#include <QMenu>
#ifdef GL2PS
#include <gl2ps.h>
#endif
#include "model.h"
#include "scoped_timer.h"

#ifndef GL_MULTISAMPLE//anti-aliasing
#define GL_MULTISAMPLE 0x809D
#endif

const double PI= 3.14159265358979323846;

template<typename PointType>
inline bool validXYZ(const PointType& p, float max_depth){
    if(max_depth < p.z)return false;
    return std::isfinite(p.z) && std::isfinite(p.y) && std::isfinite(p.x);
}

template<typename PointType>
inline void setGLColor(const PointType& p){
    unsigned char b,g,r;
    b = *(0 + (unsigned int*)(&p.data[3]));
    g = *(1 + (unsigned int*)(&p.data[3]));
    r = *(2 + (unsigned int*)(&p.data[3]));
    glColor3ub(r,g,b);
}
GLViewer::GLViewer(QWidget *parent)
        :QGLWidget(QGLFormat(QGL::NoSampleBuffers),parent),
         polygon_mode(GL_FILL),
         cloud_list_indices(),
         cloud_matrices(new QList<QMatrix4x4>()),
         show_poses_(ParameterSetting::instance()->get<bool>("show_axis")),
         show_ids_(false),
         show_grid_(false),
         show_tfs_(false),
       //show_edges_(ParameterServer::instance()->get<bool>("show_axis")),
         show_clouds_(true),
         show_octomap_(true),
         show_features_(false),
         follow_mode_(true),
         //stereo_(false),
         black_background_(true),
         width_(0),
         height_(0),
        // stereo_shift_(0.1),
         fov_(100.0/180.0*PI),
         rotation_stepping_(1.0),
         myparent(parent),
         button_pressed_(false),
         non_interactive_update_(false),
         fast_rendering_step_(1),
         external_renderable(NULL)

{
  this->initialPosition();
  this->format().setSwapInterval(0);
  back_ground_[0] = back_ground_[1] = back_ground_[3] = 0.01;//black background
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  viewpoint_tf_.setToIdentity();
}
GLViewer::~GLViewer(){}

void GLViewer::initialPosition(){
    xRot = 180 * 16.0;
    yRot = 0;
    zRot = 0;
    xTra = 0;
    yTra = 0;
    zTra = -50;
}

QSize GLViewer::minimumSizeHint() const {
    return QSize(450,450);
}

QSize GLViewer::sizeHint() const {
    return QSize(640,480);
}
static void qNormalizeAngle(int &angle) {
    while(angle < 0)
        angle += 360 * 16;
    while(angle > 360 *16)
        angle -= 360 * 16;
}
void GLViewer::setXRotation(int angle) {
    qNormalizeAngle(angle);
    if (angle != xRot) {
        xRot = angle;

    }
}
void GLViewer::setYRotation(int angle) {
    qNormalizeAngle(angle);
    if (angle != yRot) {
        yRot = angle;

    }
}

void GLViewer::setZRotation(int angle) {
    qNormalizeAngle(angle);
    if (angle != zRot) {
        zRot = angle;

    }
}

void GLViewer::setRotationGrid(double rot_step_in_degree) {
  rotation_stepping_ = rot_step_in_degree;
}


void GLViewer::initializeGL(){
    glClearColor(back_ground_[0], back_ground_[1], back_ground_[2],back_ground_[3]);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_LINE_SMOOTH);
    glDisable(GL_LIGHTING);
}
inline void GLViewer::drawTriangle(const point_type& p1, const point_type& p2, const point_type& p3)
{
    setGLColor(p1);
    glVertex3f(p1.x, p1.y , p1.z);

    setGLColor(p1);
    glVertex3f(p1.x, p1.y , p1.z);

    setGLColor(p1);
    glVertex3f(p1.x, p1.y , p1.z);


}

void GLViewer::drawGrid(){
    glBegin(GL_LINES);
    glLineWidth(1);

    ParameterSetting* ps = ParameterSetting::instance();
    float scale = ps->get<double>("gl_cell_size");
    glColor3f(0.5,0.5,0.5);
    int half_size = ps->get<int>("gl_grid_size_xy")/2;
    //xy_grid
    for(int i = -half_size; i<= half_size; i++){
        glVertex3f(i*scale, half_size*scale, 0);
        glVertex3f(i*scale, -half_size*scale, 0);//y_line

        glVertex3f(half_size*scale, i*scale, 0);
        glVertex3f(-half_size*scale, i*scale,0);//x_line

    }
    //xz_grid
    half_size = ps->get<int>("gl_grid_size_xz")/2;
    for(int i = -half_size; i<= half_size; i++){
        glVertex3f(i*scale, 0, scale*half_size);
        glVertex3f(i*scale, 0, -scale*half_size);//z_line

        glVertex3f(scale*half_size, 0, i*scale);
        glVertex3f(-scale*half_size, 0, i*scale);//x_line
    }
    //yz_grid
    half_size = ps->get<int>("gl_grid_size_yz")/2;
    for(int i = -half_size; i <= half_size; i++){
        glVertex3f(0, i*scale, scale*half_size);
        glVertex3f(0, i*scale, -scale*half_size);//z_line

        glVertex3f(0, scale*half_size, i*scale);
        glVertex3f(0, -scale*half_size, i*scale);//y_line
    }
    glEnd();
}

void GLViewer::drawAxes(float scale, float thickness){
    glEnable(GL_BLEND);
    glBegin(GL_LINES);//DRAW THE XYZ AXES
    glLineWidth(thickness);
    glColor4f(0, 0.9, 0, 1.0);//x
    glVertex3f(0, 0, 0);//x_begin
    glColor4f(0, 0.9, 0, 1.0);
    glVertex3f(scale, 0, 0);//x_end

    glColor4f(0.9, 0, 0, 1.0);//y
    glVertex3f(0, 0, 0);//y_begin
    glColor4f(0.9, 0, 0, 0);
    glVertex3f(0,scale,0);//y_end

    glColor4f(0, 0, 0.9, 1.0);//z
    glVertex3f(0, 0, 0);//z_begin
    glColor4f(0, 0, 0.9, 0);
    glVertex3f(0, 0, scale);

    glEnd();
}

void GLViewer::makeCurrent(){
    ScopedTimer s(__FUNCTION__);
    if(context() != context()->currentContext()){
        ScopedTimer s("QGLWidget::makeCurrent");
        QGLWidget::makeCurrent();
    }
}

void GLViewer::paintGL(){
    ParameterSetting* ps = ParameterSetting::instance();
    if(!this-> isVisible()) return;
    ScopedTimer s(__FUNCTION__);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPointSize(ps->get<double>("gl_point_size"));
    if(cloud_matrices->size() == 0){
        glColor4f(1,0,0,0.9);
        this->renderText(-0.54,0.1,1,QString("K"), QFont("Times New Roman",28, QFont::Bold, false));
        glColor4f(0.5,0,0.5,1);
        this->renderText(-0.45,0.1,1,QString("I"), QFont("Times New Roman",28, QFont::Bold, false));
        glColor4f(0,0,1,0.9);
        this->renderText(-0.036,0.1,1,QString("N"), QFont("Times New Roman",28, QFont::Bold, false));
        glColor4f(0,0.5,0.5,0.9);
        this->renderText(-0.027,0.1,1,QString("E"), QFont("Times New Roman",28, QFont::Bold, false));
        glColor4f(0.5,0.5,0.5,0.9);
        this->renderText(-0.18,0.1,1,QString("C"), QFont("Times New Roman",28, QFont::Bold, false));
        glColor4f(00,1,0,0.9);
        this->renderText(-0.09,0.1,1,QString("T"), QFont("Times New Roman",28, QFont::Bold, false));
        glColor4f(1-back_ground_[0], 1-back_ground_[1],1-back_ground_[2],1.0);
        this->renderText(0.01,0.1,1, QString("S"), QFont("Sans",  28, -1,false));//slam
        this->renderText(0.08,0.1,1, QString("L"), QFont("Sans",  28, -1,false));
        this->renderText(0.15,0.1,1, QString("A"), QFont("Sans",  28, -1,false));
        this->renderText(0.23,0.1,1, QString("M"), QFont("Sans",  28, -1,false));
    }
    drawClouds();
    drawRenderable();
}

void GLViewer::drawRenderable() {
  if(show_octomap_ && external_renderable != NULL){
    external_renderable->render();
  }
}
//draw i th pointcloud
void GLViewer::drawOneCloud(int i){
    glPushMatrix();
    glMultMatrixf(static_cast<GLfloat*>((*cloud_matrices)[i].data()));
    if(show_clouds_) glCallList(cloud_list_indices[i]);//pose * pointcloud
    if(show_features_ && feature_list_indices.size() > i){
        glCallList(feature_list_indices[i]);

    }

    glPopMatrix();
}

void GLViewer::drawNavigationAxis(int axis_idx, float scale, QString text){
    float coords[3] = {0.0, 0.0 ,0.0};
    float colors[3] = {1.0, 1.0 ,1.0};
    colors[axis_idx] = 0.0;

    glEnable(GL_BLEND);
    glBegin(GL_LINES);

    glColor3fv(colors);

    coords[axis_idx] = scale * 10;
    glVertex3fv(coords);

    coords[axis_idx] = -scale * 10;//extend the axis when press the button
    glVertex3fv(coords);

    glEnd();
    coords[axis_idx] = -scale;
    this->renderText(coords[0],coords[1]+0.01,coords[2],text, QFont("Monospace", 8));

}

inline void GLViewer::clearAndUpdate(){
  ScopedTimer s(__FUNCTION__);
  makeCurrent();
  paintGL();
  if(this->format().doubleBuffer())
  {
    ScopedTimer s("SwapBuffers");
    swapBuffers();
  }
}

void GLViewer::drawClouds(){
    ScopedTimer(__FUNCTION__);
    ParameterSetting* ps = ParameterSetting::instance();
    if(follow_mode_){
        int id = cloud_matrices->size()-1;
        if(id >= 0)ViewPoint((*cloud_matrices)[id]);
    }
    glDisable(GL_BLEND);//CLOUDS doesnt have the alpha
    glLoadIdentity();//load the matrix to record the movment
    //camera transformation
    glTranslatef(xTra, yTra, zTra);//translation
    if(button_pressed_){
        drawNavigationAxis(0, 0.5, "mouse up/down");
    }
    int x_steps = (xRot / 16.0)/rotation_stepping_;
    glRotatef(x_steps * rotation_stepping_, 1.0, 0.0, 0.0);

    if(button_pressed_){
        drawNavigationAxis(1, 0.5, "mouse left/right");
    }
    int y_steps = (yRot / 16.0)/rotation_stepping_;
    glRotatef(y_steps * rotation_stepping_, 0.0, 1.0, 0.0);

    if(button_pressed_){
        drawNavigationAxis(2, 0.5, "ctrl+left/right");
    }
    int z_steps = (zRot / 16.0)/rotation_stepping_;
    glRotatef(z_steps *rotation_stepping_,0.0, 0.0 ,1.0);

    glMultMatrixf(static_cast<GLfloat*>( viewpoint_tf_.data() ));
    if(show_grid_){
        drawGrid();//10x10 grid 1x1 cells
    }
    if(show_poses_) drawAxes(0.5);//0~0.5 xyz
    ROS_DEBUG("Drawing %i PointClouds", cloud_list_indices.size());
    //begin drawing pointcloud
    int step = 1;
    if(button_pressed_ || non_interactive_update_){
        step = std::max(step, (int)fast_rendering_step_);
        step = std::max(step,ps->get<int>("fast_rendering_step"));
    }
    //get the pose and the last pointcloud in the cache
    int last_cloud = std::min(cloud_list_indices.size(),cloud_matrices->size());

    //just one cloud
    int specific_cloud = ps->get<int>("show_cloud_with_id");
    if(specific_cloud >= 0){
        drawOneCloud(specific_cloud);
    }
    else//show all pointcloud
    {   //first 0~last_cloud
        for(int i=0;i < last_cloud - 10; i+= step){
            ROS_DEBUG("Drawing %d.th PointCloud.",i);
            drawOneCloud(i);
        }//
        for(int j = std::max(last_cloud - 10, 0); j< last_cloud; j+= step){
            ROS_DEBUG("Drawing %d.th PointCloud.",j);
            drawOneCloud(j);
        }
    }
     glDisable(GL_DEPTH_TEST);
     //if(show_edges_) drawEdges();

     //then we can make the Axes tansform with the camera-poincloud pose
     for(int i = 0; i<cloud_list_indices.size() && i<cloud_matrices->size();i++){
         glPushMatrix();
         glMultMatrixf(static_cast<GLfloat*>( (*cloud_matrices)[i].data() ));

         if(show_poses_) drawAxes((i + 1 == cloud_list_indices.size() ? 0.5:0.075));//last poses bigger
         if(show_ids_)
         {
                   glColor4f(1-back_ground_[0],1-back_ground_[1],1-back_ground_[2],1.0); //inverse of bg color
                   this->renderText(0.,0.,0.,QString::number(i), QFont("Monospace", 8));
         }
                   glPopMatrix();
        }
        glEnable(GL_DEPTH_TEST);
     }



void GLViewer::resizeGL(int width, int height)
{
    width_ = width;
    height_ =height;

    glViewport(0,0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float ratio = (float)width /(float) height;
    gluPerspective(fov_, ratio, 0.1, 1e4);

    glMatrixMode(GL_MODELVIEW);
}

void GLViewer::mouseDoubleClickEvent(QMouseEvent *event)
{
    this->initialPosition();
    if(cloud_matrices->size() > 0){
        int id = 0;
        switch(QApplication::keyboardModifiers() ){

        case Qt::NoModifier:
              id = cloud_matrices->size() - 1;//last pose
              follow_mode_ = true;
              ViewPoint((*cloud_matrices)[id]);
            break;
        case Qt::ControlModifier:
            follow_mode_ = false;
            ViewPoint((*cloud_matrices)[id]);//first pose
            break;
        case Qt::ShiftModifier:
            viewpoint_tf_.setToIdentity();//origin pose

        }
    }
    if(setClickedPosition(event->x(), event->y())){
        follow_mode_ = false;
    }
    clearAndUpdate();

}

void GLViewer::toggleBackgroundColor(bool on){
    black_background_ = on;
    if(on){
        back_ground_[0] = back_ground_[1] = back_ground_[2] = back_ground_[3] = 0.01;
    }
    else{
        back_ground_[0] = back_ground_[1] = back_ground_[2] = 1.0;
    }
    glClearColor(back_ground_[0], back_ground_[1], back_ground_[2], back_ground_[3]);
    clearAndUpdate();
}

void GLViewer::toggleShowFeatures(bool on){
    show_features_ = on;
    clearAndUpdate();
}

void GLViewer::toggleShowClouds(bool on){
    show_clouds_ = on;
    clearAndUpdate();
}

void GLViewer::toggleShowGrid(bool on){
    show_grid_ = on;
    clearAndUpdate();
}

void GLViewer::toggleShowIDs(bool on){
    show_ids_ = on;
    clearAndUpdate();
}

void GLViewer::toggleFollowMode(bool on){
    follow_mode_ = on;
}

/*context menu*/
void GLViewer::mouseReleaseEvent(QMouseEvent *event){
    button_pressed_ = false;
    clearAndUpdate();
    if(event->button() == Qt::RightButton)
    {
        QMenu menu;
        QMenu* viewMenu = &menu;
        GLViewer* glviewer = this;

        QAction* toggleShowPointCloud = new QAction(tr("Show &Cloud"),this);
        toggleShowPointCloud->setCheckable(true);
        toggleShowPointCloud->setChecked(show_clouds_);
        toggleShowPointCloud->setStatusTip(tr("Toggle whether point clouds should be displayed"));
        connect(toggleShowPointCloud,SIGNAL(toggled(bool)),glviewer,SLOT(toggleShowClouds(bool)));
        viewMenu->addAction(toggleShowPointCloud);

        QAction *toggleShowFeatures = new QAction(tr("Show &feature Location"),this);
        toggleShowFeatures->setCheckable(true);
        toggleShowFeatures->setChecked(show_features_);
        toggleShowFeatures->setStatusTip(tr("Toggle whether features location should be displayed"));
        connect(toggleShowFeatures,SIGNAL(toggled(bool)),glviewer,SLOT(toggleShowFeatures(bool)));
        viewMenu->addAction(toggleShowFeatures);

        QAction *toggleFollowAct = new QAction(tr("Follow &Camera"),this);
        toggleFollowAct->setShortcut(QString("Shift+F"));
        toggleFollowAct->setCheckable(true);
        toggleFollowAct->setChecked(follow_mode_);
        toggleFollowAct->setStatusTip(tr("Always use viewpoint of last frame (except zoom)"));
        connect(toggleFollowAct, SIGNAL(toggled(bool)), glviewer, SLOT(toggleFollowMode(bool)));
        viewMenu->addAction(toggleFollowAct);

        QAction *toggleShowGrid = new QAction(tr("Show Grid"),this);
        toggleShowGrid->setCheckable(true);
        toggleShowGrid->setChecked(show_grid_);
        toggleShowGrid->setStatusTip(tr("show the xy plane grid"));
        connect(toggleShowGrid,SIGNAL(toggled(bool)),glviewer,SLOT(toggleShowGrid(bool)));
        viewMenu->addAction(toggleShowGrid);

        viewMenu->exec(mapToGlobal(event->pos()));


    }
    QGLWidget::mouseReleaseEvent(event);
}

void GLViewer::mousePressEvent(QMouseEvent *event){
    button_pressed_ = true;
    lastPos = event ->pos();
}

void GLViewer::wheelEvent(QWheelEvent* event){
    double size;
    switch(QApplication::keyboardModifiers()){
     case Qt::ControlModifier:
        size =ParameterSetting::instance()->get<double>("gl_point_size");
        size = std::max(1.0, size + event ->delta()/120.0);
        ParameterSetting::instance()->set<double>("gl_point_size",size);
        break;
     case Qt::ShiftModifier:
     case Qt::NoModifier:
     default:
        zTra +=(-zTra/50.0) * ((float)event -> delta()) /25.0;//zoom the z axes
    }
    clearAndUpdate();
}

void GLViewer::mouseMoveEvent(QMouseEvent *event){
    int dx = event->x() - lastPos.x();
    int dy = event->y() -lastPos.y();

    if(event->buttons() & Qt::LeftButton){
        switch(QApplication::keyboardModifiers()){
          case Qt::NoModifier:
             setXRotation(xRot - 5 * dx);
             setYRotation(yRot + 5 * dy);
          case Qt::ControlModifier:
             setXRotation(xRot - 5 * dx);
             setZRotation(zRot + 5 * dx);
          case Qt::ShiftModifier:
             xTra += (-zTra/50.0) * dx/200.0;
             yTra += (-zTra/50.0) * dy/200.0;
        }
        clearAndUpdate();clearAndUpdate();
    }else if(event->buttons() & Qt::MidButton){
        xTra += (-zTra/50.0) * dx/200.0;
        yTra += (-zTra/50.0) * dy/200.0;
        clearAndUpdate();
    }
    lastPos = event ->pos();
}

void GLViewer::updateTransforms(QList<QMatrix4x4> *transforms){
    ROS_WARN_COND(transforms->size() < cloud_matrices->size(), "Got less transforms than before!");
    QList<QMatrix4x4>* cloud_matrices_tmp = cloud_matrices;
    cloud_matrices = transforms;
    ROS_DEBUG("New Cloud matrices size: %d", cloud_matrices->size());
    delete cloud_matrices_tmp;
}

void GLViewer::setRenderable(Renderable* r){
  ROS_INFO("Setting Renderable");
  external_renderable = (Renderable*)r;
}

inline float squaredEuclideanDistance(point_type p1, point_type p2){
  float dx = p1.x - p2.x;
  float dy = p1.y - p2.y;
  float dz = p1.z - p2.z;
  return dx*dx + dy*dy + dz*dz;
}

//to emit a signal to add a new pointcloud
void GLViewer::addPointCloud(pointcloud_type * pc, QMatrix4x4 transform){
    ROS_DEBUG("pc pointer in addPointCloud: %p (this is %p in thread %d)", pc, this, (unsigned int)QThread::currentThreadId());
    ParameterSetting* ps=ParameterSetting::instance();
    pointCloud2GLPoints(pc);
    cloud_matrices->push_back(transform);
    Q_EMIT cloudRendered(pc);
    non_interactive_update_ = true;
    ScopedTimer s("Rendering",false);
    clearAndUpdate();

    if(s.elapsed()>0.05){
        fast_rendering_step_++;
        ROS_INFO("Increased renderer skip to every %d",fast_rendering_step_);
    }else if(s.elapsed()<0.01 && fast_rendering_step_>1){
        fast_rendering_step_--;
        ROS_INFO("Decreased renderer skip to every %d",fast_rendering_step_);
    }else
        ROS_INFO("No change to renderer skip (%d).",fast_rendering_step_);
    non_interactive_update_ = false;

}

void GLViewer::addFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > *feature_locations_3d)
{
    ParameterSetting* ps = ParameterSetting::instance();
    ROS_DEBUG("Making GL list from feature points");
    GLuint feature_list_index = glGenLists(1);
    if(!feature_list_index)
    {
        ROS_ERROR("No display list could be created");
        return;
    }
    glNewList(feature_list_index,GL_COMPILE);//create a new display list
    feature_list_indices.push_back(feature_list_index);
    glLineWidth(3*ps->get<double>("gl_point_size"));
    glBegin(GL_LINES);
    float r = (float)rand()/(float)RAND_MAX;//random color
    float g = (float)rand()/(float)RAND_MAX;
    float b = (float)rand()/(float)RAND_MAX;
    //for_each draw the std error
    BOOST_FOREACH(const Eigen::Vector4f& tf, *feature_locations_3d)
    {
         if(std::isfinite(tf[3]))
         {
             glColor4f(r,g,b,1.0);//lucency
             glVertex3f(tf[0],tf[1],tf[2]);
             glColor4f(r,g,b,0.0);
             glVertex3f(tf[0],tf[1],tf[2]-depth_std_mean(tf[2]));
         }
    }
    glEnd();
    glLineWidth(1.0);
    glEndList();
}
void GLViewer::pointCloud2GLPoints(pointcloud_type *pc){
    ScopedTimer s(__FUNCTION__);
    ROS_DEBUG("Making GL list from point-cloud pointer %p in thread %d", pc, (unsigned int)QThread::currentThreadId());
    GLuint cloud_list_index = glGenLists(1);
    if(!cloud_list_index){
        ROS_ERROR("NO display list");
        return;
    }
    cloud_list_indices.push_back(cloud_list_index);
    glNewList(cloud_list_index,GL_COMPILE);
    glBegin(GL_POINTS);
    point_type origin;
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;

    const float max_depth = ParameterSetting::instance()->get<double>("maximum_depth");
    float depth;
    unsigned int w = pc->width, h = pc->height;
    for(unsigned int x = 0; x < w; x++){//row to col
        for(unsigned int y = 0; y < h; h++){
            using namespace pcl;
            const point_type* p = &pc->points[x+y*w];
            if(!(validXYZ(*p, max_depth))) continue;
            setGLColor(*p);
            glVertex3f(p->x, p->y, p->z);
        }
    }
    glEnd();
    ROS_DEBUG("Compiled pointcloud into list %i",  cloud_list_index);
    glEndList();
}
void GLViewer::reset(){
    if(!cloud_list_indices.empty()){
      unsigned int max= cloud_list_indices.size() > feature_list_indices.size()? cloud_list_indices.back() : feature_list_indices.back();
      glDeleteLists(1,max);
    }
    cloud_list_indices.clear();
    feature_list_indices.clear();
    cloud_matrices->clear();
    clearAndUpdate();
}

QImage GLViewer:: renderList(QMatrix4x4 transform, int list_id){
    return QImage();
}

void GLViewer::ViewPoint(QMatrix4x4 cam_pos){
    //Moving the camera is inverse to moving the points to draw
    viewpoint_tf_ = cam_pos.inverted();
}

bool GLViewer::setClickedPosition(int x, int y){
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX,winY,winZ;//windows xyz
    GLdouble posX, posY, posZ;

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX,projection);
    glGetIntegerv(GL_VIEWPORT,viewport);

    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels(x,int(winY),1,1,GL_DEPTH_COMPONENT,GL_FLOAT,&winZ);
    if(winZ != 1){
        gluUnProject(winX,winY,winZ,modelview,projection,viewport,&posX,&posY,&posZ);
        ROS_INFO_STREAM((float)winZ << ", [" << posX << "," << posY << "," << posZ << "]");
        viewpoint_tf_.setToIdentity();
        viewpoint_tf_(0,3) = -posX;
        viewpoint_tf_(1,3) = -posY;
        viewpoint_tf_(2,3) = -posZ;
        Q_EMIT clickPosition(posX, posY, posZ);
        return true;
    }
    else{
        return false;
    }
}
void GLViewer::deleteLastNode(){
  if(cloud_list_indices.size() <= 1){
    this->reset();
    return;
  }
    GLuint nodeId = cloud_list_indices.back();//获得节点列表最后一个元素
    cloud_list_indices.pop_back();//出栈
    glDeleteLists(nodeId,1);//删除列表
    GLuint ftId = feature_list_indices.back();//获得特征点id
    feature_list_indices.pop_back();//出栈
    glDeleteLists(ftId,1);//删除列表
}


