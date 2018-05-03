#ifndef GLVIEWER_H
#define GLVIEWER_H
#include <QGLWidget>
#include <QList>
#include <QPair>
#include <QMatrix4x4>
#include "parameter_setting.h"
//abstract class
class Renderable {
  public:
  virtual void render() = 0;
};


class GLViewer: public QGLWidget{
  Q_OBJECT
public:
    GLViewer(QWidget *parent = 0);
    ~GLViewer();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;//QWidget preferred size

public Q_SLOTS:
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);
  //void setSereoShift(double shift);
    void setRotationGrid(double shift);
    void toggleFollowMode(bool on);
  //void toogleShowEdges(bool on);
    void toggleShowClouds(bool on);
    void toggleShowFeatures(bool on);
    void toggleShowGrid(bool on);
    void toggleShowIDs(bool on);
    void toggleBackgroundColor(bool on);
  //void toggleStereo(bool on);

    void addPointCloud(pointcloud_type * pc, QMatrix4x4 transform);
    void addFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >* feature_locations_3d);//加入特征
    void updateTransforms(QList<QMatrix4x4>* transforms);
   // void setEdges(const QList<QPair<int, int> >* edge_list);
    void deleteLastNode();
    void reset();
   // void toggleTriangulation();
    //void drawToPS(QString filname);
    void setRenderable(Renderable* r);
Q_SIGNALS:
    void cloudRendered(pointcloud_type*);
    void clickPosition(float x, float y, float z);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    //mouse event
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);

    void drawAxes(float scale, float thickness = 40);
    //void drawEdges();
    void drawGrid();
    void drawTriangle(const point_type& p1,const point_type& p2,const point_type& p3);
    //using GL_TRIANGLES
    //void pointCloud2GlTriangleList(pointcloud_type const *pc);
    //using GL_POINT
    void pointCloud2GLPoints(pointcloud_type * pc);
    //using GL_TRIANGLE_STRIPs
    //void pointCloud2GLStrip(pointcloud_type * pc);
    QImage renderList(QMatrix4x4 transform, int list_id);
    void drawOneCloud(int i);
    void drawClouds();
    float back_ground_[4];
    void makeCurrent();

    void initialPosition();
    void drawNavigationAxis(int axis_idx,float scale, QString text);
    void drawRenderable();

private:
    void clearAndUpdate();
    int xRot, yRot, zRot;
    float xTra ,yTra, zTra;
    QPoint lastPos;
    GLenum polygon_mode;
    QList<GLuint> cloud_list_indices;
    QList<GLuint> feature_list_indices;
    //QList<QPair<int, int> > edge_list_;
    QList<QMatrix4x4>* cloud_matrices;
    QMatrix4x4 viewpoint_tf_;

     void ViewPoint(QMatrix4x4 cam_pos);
     bool setClickedPosition(int x, int y);
     bool show_poses_;
     bool show_ids_;
     bool show_grid_;
     bool show_tfs_;
   //bool show_edges_;
     bool show_clouds_;
     bool show_octomap_;
     bool show_features_;
     bool follow_mode_;
   //bool setero_;
     bool black_background_;
     int width_, height_;
     //double stereo_shift_,
     double fov_;
     double rotation_stepping_;

     QWidget* myparent;
     bool button_pressed_;
     bool non_interactive_update_;
     unsigned int fast_rendering_step_;
     Renderable* external_renderable;

};


#endif
