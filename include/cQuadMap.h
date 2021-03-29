#ifndef MAP_FACTORY_H
#define MAP_FACTORY_H
#include <vector>
#include <math.h>
#include <GL/glu.h>
#include "quadmap/include/mapBase.h"
#include "quadmap/include/QuadTree.h"
#include <opencv2/core/core.hpp>
#include "octomap/include/octomap.h"

// using namespace std;
using namespace Eigen;
// using namespace quadmap;

#define Min(a,b) ((a) < (b) ? (a) : (b))
#define Max(a,b) ((a) < (b) ? (a) : (b))

typedef struct tagExpandInfo
{
    quadmap::point2d center;
    int iDiv;          //0:root as child; 2: childs 2 div to new tree; 4: childs 4 div to new tree
    uint8_t childID[2]; //div = 0:new childID; 2:new node1 childID and old subchildID ; 4: new 4 subchildID, child same to old childID
    // uint8_t childID_2; //div = 0: invalid; 2:new node2 childID and old subchildID ; 4: invalid
    tagExpandInfo()
    : iDiv(-1)
    // , childID(0xff)
    // , childID_2(0xff)
    {
        memset(childID, 0xff,sizeof(childID));
    }
}ExpandInfo_t;
//encode:
//div = 0: childID_1: 4~5bit new childID, old childID is root;
//div = 2: childID_1: 4~5bit new childID_min, 0~1bit old subchildID_min, 2~3bit new subchildID_max;
//         childID_2: 4~5bit new childID_max, 0~1bit old sibchildID_min, 2~3bit new subchildID_max;
//div = 4: childID_1: 0~1bit new subchildID1, 2~3bit new subchildID2, 4~5bit new subchildID3, 6~7bit new subchildID4;

//decode:
//div = 0: childID = (childID_1 & 0xF0) >> 4
//div = 2: childID(j) = (childID_j & 0xF0) >> 4, subchildID(i) = ((childID_j>>2i) & 0x03)
//div = 4: old subchildID i = 0,1,2,3; new subchildID(i) = ((childID_1>>2i) & 0x03)

typedef enum{
    RECT_EXP_MARK      = 0x0F, // all direction
    RECT_EXP_LEFT      = 0x0A, // dir 1,3
    RECT_EXP_RIGHT     = 0x05, // dir 0,2
    RECT_EXP_DOWN      = 0x0C, // dir 2,3
    RECT_EXP_UP        = 0x03, // dir 0,1
    RECT_EXP_LEFTDOWN  = 0x01, // dir 0,1
    RECT_EXP_RIGHTDOWN = 0x02, // dir 0,1
    RECT_EXP_LEFTUP    = 0x04, // dir 0,1
    RECT_EXP_RIGHTUP   = 0x08, // dir 0,1
}EN_RectExpDir; //new


typedef quadmap::QuadTreeNode QuadNode;

class QuadTreeNode;
class MapFactory
{
public:
    MapFactory();
    ~MapFactory();

    //IF
    //update Map by 3D point cloud
    void UpdateMap(const std::vector<cv::Vec3d>& wdPoints, bool Is3D, int scale); //true: 3D, false:2D; scale: sacle of resolution

    //set ultrasonic waves data
    void SetUSData(bool bShowUS, const std::vector<cv::Vec3d>& USPts, const std::vector<int>& PtTypes);

    bool GetMapImage(unsigned char* imageArray, const ImageInfo_t& ImageInfo);

    //draw 3D map by cube data
    void GenerateCubes();

    void DrawCubeMap();

    // //get Map by input scope
    // bool GetMapAll(Map2D_t& map2D);

    cv::Matx44d GetCarOrig(){ return m_carOrig; }

private:
    // void init();
    void init(bool Is3D);
    void loadCarOrig();

    //for quadtree scop for large(more than 50 meters)
    bool ExpandTree(Rectangle_t& cloudRect);

    bool IsCouldOverflowTreeScope(Rectangle_t& cloudRect, Rectangle_t& treeRect);

    void UpdateTreeRect(const quadmap::point2d& tree_center, double tree_width);

    void getExpandInfo(const Rectangle_t& cloudRect, ExpandInfo_t& expandInfo);

    void outQuadTree(std::string outPath);

    //get Map by input scope not delete
    // bool GetMapByScop(Rectangle_t& stRect, Map2D_t& map2D);

    bool GetValidScop(const Rectangle_t& ImageRect, Rectangle_t& ValidScope);

    //3D cube data
    void initCubeTemplate(std::vector<octomath::Vector3>& cube_template);

    //add one cube to arrays
    unsigned int generateCube(const octomap::OcTreeVolume& v,
                            const std::vector<octomath::Vector3>& cube_template,
                            const unsigned int& current_array_idx,
                            GLfloat*** glArray);

    void initGLArrays(const unsigned int& num_cubes,
                    unsigned int& glArraySize,
                    GLfloat*** glArray);

    void clearCubes(GLfloat*** glArray,
                    unsigned int& glArraySize);

    void drawOccupiedVoxels() const;

    void drawCubes(GLfloat** cubeArray, unsigned int cubeArraySiz) const;

    void GeneratePtsCube(const std::vector<cv::Vec3d>& QuadPts); //2D

    void GenerateOcTreeCube();

    bool GenerateQuadTreeCube(  quadmap::QuadTree* pQuadTree,
                                unsigned int& glArraySize,
                                GLfloat*** glArray);

    void GetCubePoints(std::vector<cv::Vec3d>& QuadPts); //Cube or Grid center point

    //get ultrasonic Unoccupied vexel.
    void GetUSUnoccupiedPts(quadmap::QuadTree* pQuadTree,        //[out]
                            Rectangle_t& UsScope,                 //[out]
                            const std::vector<cv::Vec3d>& USPts, //[in]
                            const std::vector<int>& PtTypes);     //[in]
    void GenerateUSPtsCube(const std::vector<octomap::point3d>& USPts);

private:

    Rectangle_t         m_treeScope;
    quadmap::QuadTree*  m_pQuadTree;
    octomap::OcTree*    m_pOctoTree;
    double              m_resolution;

    //show map in view
    bool                m_bIs3D;
    cv::Matx44d         m_carOrig;
    double              m_alphaOccupied;
    GLfloat**           m_occupiedThresArray;
    unsigned int        m_occupiedThresSize;
    GLfloat**           m_occupiedArray;
    unsigned int        m_occupiedSize;

    // Unoccupied for ultrasonic obstcale space.
    GLfloat**           m_USUnoccupiedArray;
    unsigned int        m_USUnoccupiedSize;
};

#endif //MAP_FACTORY_H
