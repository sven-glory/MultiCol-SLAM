#include "quadmap/include/QuadTreeNode.h"
#include "cQuadMap.h"
#include <iomanip>
// #include <qglviewer.h>
#define MAP_GROUND_HIGHT  0.05
#define ULTRASONIC_UNIT   0.1

MapFactory::MapFactory()
    : m_pQuadTree(NULL), m_pOctoTree(NULL)
    , m_alphaOccupied(0.8)
    , m_occupiedThresSize(0), m_occupiedSize(0), m_USUnoccupiedSize(0)
    , m_occupiedThresArray(NULL), m_occupiedArray(NULL), m_USUnoccupiedArray(NULL)
    , m_resolution(TREE_RESOLUTION)
    , m_bIs3D(true)
{
    // init();
    loadCarOrig();
}

MapFactory::~MapFactory()
{
    if (m_pQuadTree)
    {
        m_pQuadTree->clear();
        m_pQuadTree = NULL;
    }
    if (m_pOctoTree)
    {
        m_pOctoTree->clear();
        m_pOctoTree = NULL;
    }

    //release array
    clearCubes(&m_occupiedThresArray, m_occupiedThresSize);
    clearCubes(&m_occupiedArray, m_occupiedSize);
    clearCubes(&m_USUnoccupiedArray, m_USUnoccupiedSize);
}

// void MapFactory::init()
// {
//     if (m_pQuadTree)
//     {
//         m_pQuadTree->clear();
//         m_pQuadTree = NULL;
//     }
//     m_pQuadTree = new quadmap::QuadTree(m_resolution, TREE_DEPTH_INIT, quadmap::point2d(TREE_CENTER_X,TREE_CENTER_Y));
//     UpdateTreeRect(m_pQuadTree->GetTreeCenter(), m_pQuadTree->GetTreeWidth());

//     if (m_pOctoTree)
//     {
//         m_pOctoTree->clear();
//         m_pOctoTree = NULL;
//     }
//     m_pOctoTree = new octomap::OcTree(m_resolution);
// }

void MapFactory::init(bool Is3D)
{
    if (Is3D)
    {
        if (m_pOctoTree)
        {
            m_pOctoTree->clear();
            m_pOctoTree = NULL;
        }
        octomap::OcTree temMap(m_resolution);
        m_pOctoTree = new octomap::OcTree(temMap);
    }
    else
    {
        if (m_pQuadTree)
        {
            m_pQuadTree->clear();
            m_pQuadTree = NULL;
        }
        m_pQuadTree = new quadmap::QuadTree(m_resolution, TREE_DEPTH_INIT, quadmap::point2d(TREE_CENTER_X,TREE_CENTER_Y));
        UpdateTreeRect(m_pQuadTree->GetTreeCenter(), m_pQuadTree->GetTreeWidth());
    }
}

void MapFactory::loadCarOrig()
{
    //origin rectangle of car init
    double wdZ(0.5);
    const float wd = 1.83;   //car width
    const float ln = 4.12;   //car length
    const float ltX =-0.43;  //left top X
    const float ltY = 0.63;  //left top Y
    m_carOrig = cv::Matx44d::ones();
    m_carOrig(0, 0) = ltX;
    m_carOrig(1, 0) = ltY-wd;
    m_carOrig(2, 0) = wdZ;

    m_carOrig(0, 1) = ltX;
    m_carOrig(1, 1) = ltY;
    m_carOrig(2, 1) = wdZ;

    m_carOrig(0, 2) = ltX-ln;
    m_carOrig(1, 2) = ltY;
    m_carOrig(2, 2) = wdZ;

    m_carOrig(0, 3) = ltX-ln;
    m_carOrig(1, 3) = ltY-wd;
    m_carOrig(2, 3) = wdZ;
}

void MapFactory::UpdateMap(const std::vector<cv::Vec3d>& wdPoints, bool Is3D, int scale)
{
    m_resolution = TREE_RESOLUTION * scale;
    m_bIs3D      = Is3D;

    init(Is3D);

    if (Is3D)
    {
        for (size_t i = 0; i < wdPoints.size(); ++i) {
            m_pOctoTree->updateNode(octomap::point3d(wdPoints[i](0), wdPoints[i](1), wdPoints[i](2)),true);
        }
    }
    else
    {
        for (size_t i = 0; i < wdPoints.size(); ++i) {
            m_pQuadTree->updateNode( quadmap::point2d(wdPoints[i](0), wdPoints[i](1)), true );
        }
    }
}

//set ultrasonic waves data
void MapFactory::SetUSData(bool bShowUS, const std::vector<cv::Vec3d>& USPts, const std::vector<int>& PtTypes)
{
    if ( (USPts.size() != PtTypes.size()) )
    {
        return;
    }

    if (!bShowUS)
    {
        clearCubes(&m_USUnoccupiedArray, m_USUnoccupiedSize);
        return;
    }

    //get ultrasonic scope and unoccupied quad tree.
    Rectangle_t UsScope;
    quadmap::QuadTree* pUSTree = new quadmap::QuadTree(m_resolution, TREE_DEPTH_INIT, quadmap::point2d(TREE_CENTER_X,TREE_CENTER_Y));
    GetUSUnoccupiedPts(pUSTree, UsScope, USPts, PtTypes);

    GenerateQuadTreeCube(pUSTree, m_USUnoccupiedSize, &m_USUnoccupiedArray);

    //update ultrasonic Occupied and Unoccupied information.
    bool bCheck = true;
    double mapFactor = 1/m_resolution;
    unsigned int max_val_x = ceil( (UsScope.PointB.x - UsScope.PointA.x) / m_resolution );
     if (m_bIs3D)
    {
        if (m_pOctoTree)
        {
            float ProbMissLog = m_pOctoTree->getProbMissLog(); //-0.41
            unsigned int maxDepth = m_pOctoTree->getTreeDepth();
            octomap::OcTree::iterator it_bbx  = m_pOctoTree->begin(maxDepth);
            octomap::OcTree::iterator end_bbx = m_pOctoTree->end();
            it_bbx  = m_pOctoTree->begin(maxDepth);
            for (; it_bbx != end_bbx; ++it_bbx)
            {
                if (m_pOctoTree->isNodeOccupied(*it_bbx)){
                    //check is point in ultrasonic unoccupied cells.
                    octomap::point3d Pt3D = it_bbx.getCoordinate();

                    if ( pUSTree->isNodeOccupied(quadmap::point2d(Pt3D(0),Pt3D(1))) )
                    {
                        float fLogOdds = (*it_bbx).getLogOdds();
                        fLogOdds = -fLogOdds-0.01;//ceil(fLogOdds / -ProbMissLog ) * ProbMissLog;
                        m_pOctoTree->updateNode(Pt3D,fLogOdds);

                        bCheck = m_pOctoTree->isNodeOccupied(*it_bbx);
                        if (bCheck)
                        {
                            bCheck = false;//[Debug]assert
                        }

                    }
                }
            }

            for (size_t i = 0; i < USPts.size(); ++i) {
                m_pOctoTree->updateNode(octomap::point3d(USPts[i](0), USPts[i](1), MAP_GROUND_HIGHT),true);
            }
        }

    }
    else
    {
        if (m_pQuadTree)
        {
            // std::vector<quadmap::point2d> OccuPts;
            float ProbMissLog = m_pQuadTree->getProbMissLog(); //-0.41
            quadmap::point2d Pt_min = quadmap::point2d(UsScope.PointA.x, UsScope.PointA.y);
            quadmap::point2d Pt_max = quadmap::point2d(UsScope.PointB.x, UsScope.PointB.y);
            unsigned int maxDepth = m_pQuadTree->getTreeDepth();
            quadmap::QuadTree::leaf_bbx_iterator it_bbx  = m_pQuadTree->begin_leafs_bbx(Pt_min, Pt_max, maxDepth);
            quadmap::QuadTree::leaf_bbx_iterator end_bbx = m_pQuadTree->end_leafs_bbx();
            for (; it_bbx != end_bbx; ++it_bbx)
            {
                if (m_pQuadTree->isNodeOccupied(*it_bbx))
                {
                    quadmap::point2d Pt2D = it_bbx.getCoordinate();
                    if ( pUSTree->isNodeOccupied( Pt2D ) )
                    {
                        float fLogOdds = (*it_bbx).getLogOdds();
                        fLogOdds = -fLogOdds-0.01;//ceil(fLogOdds / -ProbMissLog ) * ProbMissLog;
                        m_pQuadTree->updateNode(Pt2D,fLogOdds);

                        bCheck = m_pQuadTree->isNodeOccupied(*it_bbx);
                        if (bCheck)
                        {
                            bCheck = false;//[Debug]assert
                        }
                    }
                }
            }

            for (size_t i = 0; i < USPts.size(); ++i) {
                m_pQuadTree->updateNode(quadmap::point2d(USPts[i](0), USPts[i](1)),true);
            }
        }
    }

    pUSTree->clear();
    delete pUSTree;
}

void MapFactory::GetCubePoints(std::vector<cv::Vec3d>& QuadPts)
{
    QuadPts.clear();
    if (!m_pQuadTree)
    {
        return;
    }

    bool bRet = true;

    Rectangle_t ValidScope;
    ValidScope.bIsInit = true;
    m_pQuadTree->getMetricMinMax(ValidScope.PointA.x,ValidScope.PointA.y,ValidScope.PointB.x,ValidScope.PointB.y);

    double treeResolution = m_pQuadTree->getResolution();

    //get image pixel occupied information
    //update occupied info to map2D****
    quadmap::point2d Pt_min(ValidScope.PointA.x, ValidScope.PointA.y);
    quadmap::point2d Pt_max(ValidScope.PointB.x, ValidScope.PointB.y);
    quadmap::QuadTree::leaf_bbx_iterator it_bbx  = m_pQuadTree->begin_leafs_bbx(Pt_min, Pt_max);
    quadmap::QuadTree::leaf_bbx_iterator end_bbx = m_pQuadTree->end_leafs_bbx();
    // quadmap::QuadTree::iterator it_bbx  = m_pQuadTree->begin();
    // quadmap::QuadTree::iterator end_bbx = m_pQuadTree->end();
    unsigned int maxDepth = m_pQuadTree->getTreeDepth();
    float occThresLog = m_pQuadTree->getOccupancyThresLog();
    double mapFactor = 1/treeResolution;
    int iOccupied = -1;

    int iCount = 0;
    int iCount2 = 0;
    int iCount3 = 0;
    int iCount4 = 0;
    int iCount5 = 0;
    int iNodeCnt[TREE_DEPTH_INIT+1] = {0};
    double testX(0.0),testY(0.0);
    Rectangle_t PtsScope;
    PtsScope = m_treeScope;
    double dPtsResolution = treeResolution;

    // for (quadmap::QuadTree::iterator it_bbx = m_pQuadTree->begin(); it_bbx != m_pQuadTree->end(); ++it_bbx)
    for (; it_bbx != end_bbx; ++it_bbx)
    {
        unsigned int nodeDepth = it_bbx.getDepth();
        if (nodeDepth <= maxDepth )
        {
            if (m_pQuadTree->isNodeOccupied(*it_bbx))
            {
                quadmap::point2d PtCoor = it_bbx.getCoordinate();

                int offset = 0;
                if ( nodeDepth == maxDepth )
                {
                    QuadPts.push_back( cv::Vec3d(PtCoor.x(), PtCoor.y(), MAP_GROUND_HIGHT) );
                    iCount4++;
                }
                else
                {
                    int id_x = (int)floor( ( PtCoor.x() - PtsScope.PointA.x ) * mapFactor );
                    int id_y = (int)floor( ( PtCoor.y() - PtsScope.PointA.y ) * mapFactor );

                    offset = (1 << (maxDepth - nodeDepth)) / 2;

                    int cellIdx = 0;
                    for (int iy = (id_y - offset); iy < (id_y + offset); iy++)
                    {
                        for (int ix = (id_x - offset); ix < (id_x + offset); ix++)
                        {
                            testX = PtsScope.PointA.x + (double)(ix + 0.5) * dPtsResolution;
                            testY = PtsScope.PointA.y + (double)(iy + 0.5) * dPtsResolution;

                            QuadPts.push_back( cv::Vec3d(testX, testY, MAP_GROUND_HIGHT) );

                            iCount5++;
                        }
                    }
                }
            }

            iNodeCnt[nodeDepth]++;
        }
        // iCount++;
    }
}


//world coor
// bool MapFactory::UpdateMap(POINTCLOUD& wdPoints)
// {
//     // Wd点云 ->ｙ轴消失 ->压缩(八叉树) ->覆盖

//     Rectangle_t cloudRect;
//     for (size_t i = 0; i < wdPoints.size(); ++i)
//     {
//         if (wdPoints[i].x < cloudRect.PointA.x) cloudRect.PointA.x = wdPoints[i].x;
//         if (wdPoints[i].y < cloudRect.PointA.y) cloudRect.PointA.y = wdPoints[i].y;

//         if (wdPoints[i].x > cloudRect.PointB.x) cloudRect.PointB.x = wdPoints[i].x;
//         if (wdPoints[i].y > cloudRect.PointB.y) cloudRect.PointB.y = wdPoints[i].y;
//     }

//     m_pQuadTree->GetTreeRect(m_treeScope.PointA.x, m_treeScope.PointA.y, m_treeScope.PointB.x, m_treeScope.PointB.y);


//     bool bExpandRet(true);
//     if ( IsCouldOverflowTreeScope(cloudRect,m_treeScope) )
//     {
//         if (0 == m_pQuadTree->size())
//         {
//             int addDepth = 1;

//             quadmap::point2d tree_center = m_pQuadTree->GetTreeCenter();
//             double tree_width = m_pQuadTree->GetTreeWidth();
//             for (int i = addDepth; i < TREE_DEPTH_EXPAND; i++)
//             {
//                 tree_width *= 2;
//                 UpdateTreeRect(tree_center, tree_width);
//                 if (!IsCouldOverflowTreeScope(cloudRect, m_treeScope))
//                 {
//                     addDepth = i;
//                     break;
//                 }
//             }

//             quadmap::QuadTree tree (m_resolution, TREE_DEPTH_INIT + addDepth, tree_center);
//             m_pQuadTree = new quadmap::QuadTree(tree);
//             UpdateTreeRect(m_pQuadTree->GetTreeCenter(), m_pQuadTree->GetTreeWidth());
//         }
//         else
//         {
//             bExpandRet = ExpandTree(cloudRect);
//         }
//     }

//     if (!bExpandRet)
//     {
//         return false;
//     }

//     for (size_t i = 0; i < wdPoints.size(); ++i)
//     {
//         // quadmap::point2d quadPoint(coor3D(0),coor3D(2));
//         m_pQuadTree->updateNode(quadmap::point2d(wdPoints[i].x,wdPoints[i].y),true);
//     }

//     return true;
// }

//get ultrasonic Unoccupied vexel.
void MapFactory::GetUSUnoccupiedPts(quadmap::QuadTree* pQuadTree,
                                    Rectangle_t& UsScope,
                                    const std::vector<cv::Vec3d>& USPts,
                                    const std::vector<int>& PtTypes)
{
    if (!pQuadTree || (0 == USPts.size()))
    {
        return;
    }
    // double Map2D_resolution(1.0 * 1e-1);
    double mapFactor = 1/ULTRASONIC_UNIT;

    // Ultrasonic Scope
    UsScope.PointA.x = 1.0 * 1e+6;
    UsScope.PointA.y = 1.0 * 1e+6;
    UsScope.PointB.x = -1.0 * 1e+6;
    UsScope.PointB.y = -1.0 * 1e+6;
    // double dXmin(1e+6),dXmax(-1e+6),dYmin(1e+6),dYmax(-1e+6);
    for (size_t i = 0; i < USPts.size(); ++i)
    {
        if (USPts[i](0) > UsScope.PointB.x)
            UsScope.PointB.x = USPts[i](0); //dXmax
        if (USPts[i](0) < UsScope.PointA.x)
            UsScope.PointA.x = USPts[i](0); //dXmin
        if (USPts[i](1) > UsScope.PointB.y)
            UsScope.PointB.y = USPts[i](1); //dYmax
        if (USPts[i](1) < UsScope.PointA.y)
            UsScope.PointA.y = USPts[i](1); //dYmin
    }

    //
    //Map2D Ultrasonic Scope
    Map2D_t map2D;
    map2D.scope.PointA.x = (double)(( floor(UsScope.PointA.x * mapFactor)- 1 ) * ULTRASONIC_UNIT);
    map2D.scope.PointA.y = (double)(( floor(UsScope.PointA.y * mapFactor)- 1 ) * ULTRASONIC_UNIT);
    map2D.scope.PointB.x = (double)(( ceil(UsScope.PointB.x * mapFactor) + 1 ) * ULTRASONIC_UNIT);
    map2D.scope.PointB.y = (double)(( ceil(UsScope.PointB.y * mapFactor) + 1 ) * ULTRASONIC_UNIT);
    UsScope = map2D.scope;

    //set Map2D Occupied
    map2D.scope.bIsInit = true;
    map2D.resolution = ULTRASONIC_UNIT; //unit 1.0meter per pixel
    unsigned int max_val_x = ceil( (map2D.scope.PointB.x - map2D.scope.PointA.x) * mapFactor );
    unsigned int max_val_y = ceil( (map2D.scope.PointB.y - map2D.scope.PointA.y) * mapFactor );
    map2D.cells2D.reserve(max_val_x * max_val_y);
    for (size_t iy = 0; iy < max_val_y; iy++)
    {
        for (size_t ix = 0; ix < max_val_x; ix++)
        {
            Rect2D_t RectTemp;
            RectTemp.key[0] = ix;
            RectTemp.key[1] = iy;
            RectTemp.iOccupied = 0;
            map2D.cells2D.push_back(RectTemp);
        }
    }

    std::vector<int> voccupiedIdx; //[Debug]check-9 palace grid
    for (size_t i = 0; i < USPts.size(); ++i)
    {
        int id_x = (int)floor( (USPts[i](0) - map2D.scope.PointA.x) * mapFactor );
        int id_y = (int)floor( (USPts[i](1) - map2D.scope.PointA.y) * mapFactor );

        int cellIdx = id_y * max_val_x + id_x;
        map2D.cells2D[cellIdx].iOccupied = 1;

        if  ( (voccupiedIdx.size() == 0) || (cellIdx != voccupiedIdx.back()) )
        {
            voccupiedIdx.push_back(cellIdx); //[Debug]chcheck-9 palace grid
        }
    }

    //[Debug]check-9 palace grid-------------start----------------------------------------------
    //检查以占据点为中心的九宫格中，占据点的个数是否小于3个.
    std::vector<cv::Point2i> vcheckPts;
    int id_xC(-1),id_yC(-1),cellIdx(-1);
    for (size_t i = 0; i < voccupiedIdx.size(); i++)
    {
        id_xC = map2D.cells2D[voccupiedIdx[i]].key[0];
        id_yC = map2D.cells2D[voccupiedIdx[i]].key[1];
        if ( (id_xC == 0) || (id_yC == 0) || (id_xC == max_val_x-1) || (id_yC == max_val_y -1) )
        {
            //if boundary point skip
            continue;
        }

        int iCnt(0);
        for (size_t id_y = id_yC-1; id_y < id_yC+2; id_y++)
        {
            for (size_t id_x = id_xC-1; id_x < id_xC+2; id_x++)
            {
                cellIdx = id_y * max_val_x + id_x;
                if (map2D.cells2D[cellIdx].iOccupied)
                {
                    iCnt++;
                }
            }

        }
        if (iCnt < 3)
        {
            vcheckPts.push_back(cv::Point2i(id_xC,id_yC));
        }
    }

    //修复边界间断Grid
    for (size_t i = 1; i < vcheckPts.size(); i++)
    {
        //是否在同一九宫格,如果在,修复中心位置为占据.
        int idiffX = abs(vcheckPts[i].x - vcheckPts[i-1].x);
        int idiffY = abs(vcheckPts[i].y - vcheckPts[i-1].y);
        if ( (idiffX < 3) && (idiffY < 3) )
        {
            int id_x = int(0.5 * (vcheckPts[i].x + vcheckPts[i-1].x));
            int id_y = int(0.5 * (vcheckPts[i].y + vcheckPts[i-1].y));
            int cellIdx = id_y * max_val_x + id_x;
            map2D.cells2D[cellIdx].iOccupied = 1;
        }
    }
    //[Debug]check-9 palace grid-------------end----------------------------------------------

    //set outside cell to occupied
    for (int iy = 0; iy < max_val_y; iy++)
    {
        int cellIdx(-1);
        int ixMax = 0;
        for (int ix = 0; ix < max_val_x; ix++,ixMax++){
            cellIdx = iy * max_val_x + ix;
            if (1 == map2D.cells2D[cellIdx].iOccupied){
                break;}
            else if (0 == map2D.cells2D[cellIdx].iOccupied){
                map2D.cells2D[cellIdx].iOccupied = -1;}
        }//end loop x

        if (ixMax < max_val_x-1)
        {
            for (int ix = max_val_x-1; ix > -1; ix--){
                cellIdx = iy * max_val_x + ix;
                if (1 == map2D.cells2D[cellIdx].iOccupied){
                    break;}
                else if (0 == map2D.cells2D[cellIdx].iOccupied){
                    map2D.cells2D[cellIdx].iOccupied = -1;}
            }//end loop x
        }

    }//end loop y

    for (int ix = 0; ix < max_val_x; ix++)
    {
        int cellIdx(-1);
        int iyMax = 0;
        for (int iy = 0; iy < max_val_y; iy++,iyMax++){
            cellIdx = iy * max_val_x + ix;
            if (1 == map2D.cells2D[cellIdx].iOccupied){
                break;}
            else if (0 == map2D.cells2D[cellIdx].iOccupied){
                map2D.cells2D[cellIdx].iOccupied = -1;}
        }//end loop y

        if (iyMax < max_val_y-1)
        {
            for (int iy = max_val_y-1; iy > -1; iy--){
                cellIdx = iy * max_val_x + ix;
                if (1 == map2D.cells2D[cellIdx].iOccupied){
                    break;}
                else if (0 == map2D.cells2D[cellIdx].iOccupied){
                    map2D.cells2D[cellIdx].iOccupied = -1;}
            }//end loop y
        }

    }//end loop x

    double dPtX(0.0),dPtY(0.0);
    for (size_t iy = 0; iy < max_val_y; iy++)
    {
        for (size_t ix = 0; ix < max_val_x; ix++)
        {
            int cellIdx = iy * max_val_x + ix;
            if (0 == map2D.cells2D[cellIdx].iOccupied)
            {
                dPtX = map2D.scope.PointA.x + (double)(map2D.cells2D[cellIdx].key[0] + 0.5) * map2D.resolution;
                dPtY = map2D.scope.PointA.y + (double)(map2D.cells2D[cellIdx].key[1] + 0.5) * map2D.resolution;
                pQuadTree->updateNode(quadmap::point2d(dPtX, dPtY), true);
            }
        }
    }
}


bool MapFactory::GetMapImage(unsigned char* imageArray, const ImageInfo_t& ImageInfo)
{
    bool bRet = true;

    int size = ImageInfo.width * ImageInfo.height;
    unsigned char  RGB_undefined[3] = {220,220,220}; //RGB_free[3] = {255,255,255}; RGB_occupied[3] = {0,  0,  0};
    for(int i = 0 ; i < size;i++) {
        imageArray[i*3] = RGB_undefined[0];
        imageArray[i*3+1] = RGB_undefined[0];
        imageArray[i*3+2] = RGB_undefined[0];
    }

    double widthLen  = ImageInfo.width * ImageInfo.pixel_wd;
    double heightLen = ImageInfo.height * ImageInfo.pixel_wd;
    Map2D_t map2D;
    map2D.scope.bIsInit = true;
    map2D.scope.PointA.x= -0.5 * widthLen;
    map2D.scope.PointB.x= 0.5 * widthLen;
    map2D.scope.PointA.y= (widthLen < heightLen)? (-0.5 * widthLen) : (-0.5 * heightLen);
    map2D.scope.PointB.y= (widthLen < heightLen)? (-0.5 * widthLen + heightLen) : (0.5 * heightLen);
    map2D.resolution = ImageInfo.pixel_wd; //unit 1.0meter per pixel

    // Rectangle_t ValidScope;
    Rectangle_t ValidScope;
    ValidScope.bIsInit = true;
    m_pQuadTree->getMetricMinMax(ValidScope.PointA.x,ValidScope.PointA.y,ValidScope.PointB.x,ValidScope.PointB.y);
    bRet = GetValidScop(map2D.scope, ValidScope);
    if (!bRet)
    {
        return bRet;
    }

    double treeResolution = m_pQuadTree->getResolution();

    unsigned int max_val_x = ImageInfo.width;
    unsigned int max_val_y = ImageInfo.height;
    map2D.cells2D.reserve(max_val_x * max_val_y);
    for (size_t iy = 0; iy < max_val_y; iy++)
    {
        for (size_t ix = 0; ix < max_val_x; ix++)
        {
            Rect2D_t RectTemp;
            RectTemp.key[0] = ix;
            RectTemp.key[1] = iy;
            map2D.cells2D.push_back(RectTemp);
        }
    }

    //get image pixel occupied information
    //update occupied info to map2D****
    quadmap::point2d Pt_min(ValidScope.PointA.x, ValidScope.PointA.y);
    quadmap::point2d Pt_max(ValidScope.PointB.x, ValidScope.PointB.y);
    quadmap::QuadTree::leaf_bbx_iterator it_bbx  = m_pQuadTree->begin_leafs_bbx(Pt_min, Pt_max);
    quadmap::QuadTree::leaf_bbx_iterator end_bbx = m_pQuadTree->end_leafs_bbx();
    // quadmap::QuadTree::iterator it_bbx  = m_pQuadTree->begin();
    // quadmap::QuadTree::iterator end_bbx = m_pQuadTree->end();
    unsigned int maxDepth = m_pQuadTree->getTreeDepth();
    float occThresLog = m_pQuadTree->getOccupancyThresLog();
    int mapFactor = 1/map2D.resolution;
    int iOccupied = -1;

    int iCount = 0;
    int iCount2 = 0;
    int iCount3 = 0;
    int iCount4 = 0;
    int iCount5 = 0;
    int iNodeCnt[TREE_DEPTH_INIT+1] = {0};
    double testX(0.0),testY(0.0);
    // for (quadmap::QuadTree::iterator it_bbx = m_pQuadTree->begin(); it_bbx != m_pQuadTree->end(); ++it_bbx)
    for (; it_bbx != end_bbx; ++it_bbx)
    {
        unsigned int nodeDepth = it_bbx.getDepth();
        if (nodeDepth <= maxDepth )
        {
        //     //get occupied info
            float nodeLog = it_bbx->getLogOdds();
            if (fabs(nodeLog - occThresLog) < 0.000001)
            {
                iCount2++;
            }
            //node中心点坐标，占据信息 free||occupied, node size
            iOccupied = ( nodeLog < occThresLog )? 0 : 1;

            quadmap::point2d PtCoor = it_bbx.getCoordinate();

            int offset = 0;
            if ( nodeDepth == maxDepth )
            {
                int id_x = (int)floor( (PtCoor.x() - map2D.scope.PointA.x) * mapFactor );
                int id_y = (int)floor( (PtCoor.y() - map2D.scope.PointA.y) * mapFactor );
                // assert(id_y <max_val_y);
                int cellIdx = id_y * max_val_x + id_x;
                map2D.cells2D[cellIdx].iOccupied = iOccupied;

                // testX = map2D.scope.PointA.x + (double)(map2D.cells2D[cellIdx].key[0] + 0.5) * map2D.resolution;
                // testY = map2D.scope.PointA.y + (double)(map2D.cells2D[cellIdx].key[1] + 0.5) * map2D.resolution;
                iCount3++;
            }
            else
            {
                int id_x = (int)floor( (PtCoor.x() - map2D.scope.PointA.x + 0.5*map2D.resolution) * mapFactor );
                int id_y = (int)floor( (PtCoor.y() - map2D.scope.PointA.y + 0.5*map2D.resolution) * mapFactor );

                offset = (1 << (maxDepth - nodeDepth)) / 2;
                iCount4++;

                int cellIdx = 0;
                for (int iy = (id_y - offset); iy < (id_y + offset); iy++)
                {
                    for (int ix = (id_x - offset); ix < (id_x + offset); ix++)
                    {
                        cellIdx = iy * max_val_x + ix;
                        map2D.cells2D[cellIdx].iOccupied = iOccupied;

                        // testX = map2D.scope.PointA.x + (double)(map2D.cells2D[cellIdx].key[0] + 0.5) * map2D.resolution;
                        // testY = map2D.scope.PointA.y + (double)(map2D.cells2D[cellIdx].key[1] + 0.5) * map2D.resolution;

                        iCount5++;
                    }
                }
            }

            iNodeCnt[nodeDepth]++;
        }
        iCount++;
    }

    unsigned char  RGB_occupied[3]  = {0,  0,  0};
    for (int idx = 0; idx < map2D.cells2D.size(); idx++)
    {
        if (1 == map2D.cells2D[idx].iOccupied)
        {
            imageArray[idx*3]  = RGB_occupied[0];
            imageArray[idx*3+1]= RGB_occupied[1];
            imageArray[idx*3+2]= RGB_occupied[2];
        }
    }

    // outQuadTree("/media/wendy/DATA1/Work/AutoDrive/SLAMProject/MultiCol-SLAM_new/DataSet/mapData/quadTree.txt");
    return bRet;
}

bool MapFactory::GetValidScop(const Rectangle_t& ImageRect, Rectangle_t& ValidScope)
{
    //check if image rectangle has overlap part to quadtree rectangle
    if (ValidScope.PointA.x > ImageRect.PointB.x)
    {
        return false;
    }
    if (ValidScope.PointB.x < ImageRect.PointA.x)
    {
        return false;
    }
    if (ValidScope.PointA.y > ImageRect.PointB.y)
    {
        return false;
    }
    if (ValidScope.PointB.y < ImageRect.PointA.y)
    {
        return false;
    }

    //get the overlap part between image rectangle and quadtree rectangle
    if (ValidScope.PointA.x < ImageRect.PointA.x)
    {
        ValidScope.PointA.x = ImageRect.PointA.x;
    }
    if (ValidScope.PointB.x > ImageRect.PointB.x)
    {
        ValidScope.PointB.x = ImageRect.PointB.x;
    }
    if (ValidScope.PointA.y < ImageRect.PointA.y)
    {
        ValidScope.PointA.y = ImageRect.PointA.y;
    }
    if (ValidScope.PointB.y > ImageRect.PointB.y)
    {
        ValidScope.PointB.y = ImageRect.PointB.y;
    }

    return true;
}

bool MapFactory::IsCouldOverflowTreeScope(Rectangle_t& cloudRect, Rectangle_t& treeRect)
{
    bool bIsOverflow(false);
    if ( ( cloudRect.PointA.x < treeRect.PointA.x ) || (cloudRect.PointB.x > treeRect.PointB.x)
        || ( cloudRect.PointA.y < treeRect.PointA.y ) || (cloudRect.PointB.y > treeRect.PointB.y) )
    {
        bIsOverflow = true;
    }
    return bIsOverflow;
}

void MapFactory::UpdateTreeRect(const quadmap::point2d& tree_center, double tree_width)
{
    m_treeScope.PointA.x = (double)tree_center(0)- tree_width;
    m_treeScope.PointA.y = (double)tree_center(1)- tree_width;
    m_treeScope.PointB.x = (double)tree_center(0)+ tree_width;
    m_treeScope.PointB.y = (double)tree_center(1)+ tree_width;
}

// //get Map by input scope
// bool MapFactory::GetMapByScop(Rectangle_t& stRect, Map2D_t& map2D)
// {
//     if ( map2D.resolution < 0.001)
//     {
//         return false;
//     }

//     //init map2D****
//     double treeResolution = m_pQuadTree->getResolution();
//     map2D.scope.PointA.x = (double)floor(stRect.PointA.x / treeResolution - 1) * treeResolution;
//     map2D.scope.PointA.y = (double)floor(stRect.PointA.y / treeResolution - 1) * treeResolution;

//     if (map2D.resolution > treeResolution)
//     {
//         map2D.resolution = (double)floor(map2D.resolution / treeResolution) * treeResolution;
//     }
//     else //resolution <= treeResolution
//     {
//         map2D.resolution = treeResolution;
//     }

//     unsigned int max_val_x = ceil( (stRect.PointB.x - map2D.scope.PointA.x) / map2D.resolution ) + 2;
//     unsigned int max_val_y = ceil( (stRect.PointB.y - map2D.scope.PointA.y) / map2D.resolution ) + 2;
//     map2D.scope.PointB.x = map2D.scope.PointA.x + (double)max_val_x * map2D.resolution;
//     map2D.scope.PointB.y = map2D.scope.PointA.y + (double)max_val_y * map2D.resolution;

//     map2D.cells2D.reserve(max_val_x * max_val_y);
//     for (size_t iy = 0; iy < max_val_y; iy++)
//     {
//         for (size_t ix = 0; ix < max_val_x; ix++)
//         {
//             Rect2D_t RectTemp;
//             RectTemp.key[0] = ix;
//             RectTemp.key[1] = iy;
//             map2D.cells2D.push_back(RectTemp);
//         }
//     }

//     //update occupied info to map2D****
//     quadmap::point2d Pt_min(map2D.scope.PointA.x, map2D.scope.PointA.y);
//     quadmap::point2d Pt_max(map2D.scope.PointB.x, map2D.scope.PointB.y);
//     quadmap::QuadTree::leaf_bbx_iterator it_bbx  = m_pQuadTree->begin_leafs_bbx(Pt_min, Pt_max);
//     quadmap::QuadTree::leaf_bbx_iterator end_bbx = m_pQuadTree->end_leafs_bbx();
//     // quadmap::QuadTree::iterator it_bbx  = m_pQuadTree->begin();
//     // quadmap::QuadTree::iterator end_bbx = m_pQuadTree->end();
//     unsigned int maxDepth = m_pQuadTree->getTreeDepth();
//     float occThresLog = m_pQuadTree->getOccupancyThresLog();
//     int mapFactor = 1/map2D.resolution;
//     int iOccupied = -1;

//     int iCount = 0;
//     int iCount2 = 0;
//     int iCount3 = 0;
//     int iCount4 = 0;
//     int iCount5 = 0;
//     int iNodeCnt[11] = {0};
//     double testX(0.0),testY(0.0);
//     // for (quadmap::QuadTree::iterator it_bbx = m_pQuadTree->begin(); it_bbx != m_pQuadTree->end(); ++it_bbx)
//     for (; it_bbx != end_bbx; ++it_bbx)
//     {
//         unsigned int nodeDepth = it_bbx.getDepth();
//         if (nodeDepth <= maxDepth )
//         {
//         //     //get occupied info
//             float nodeLog = it_bbx->getLogOdds();
//             if (fabs(nodeLog - occThresLog) < 0.000001)
//             {
//                 iCount2++;
//             }
//             //node中心点坐标，占据信息 free||occupied, node size
//             iOccupied = ( nodeLog < occThresLog )? 0 : 1;

//             quadmap::point2d PtCoor = it_bbx.getCoordinate();

//             int offset = 0;
//             if ( nodeDepth == maxDepth )
//             {
//                 int id_x = (int)floor( (PtCoor.x() - map2D.scope.PointA.x) * mapFactor );
//                 int id_y = (int)floor( (PtCoor.y() - map2D.scope.PointA.y) * mapFactor );
//                 // assert(id_y <max_val_y);
//                 int cellIdx = id_y * max_val_x + id_x;
//                 map2D.cells2D[cellIdx].iOccupied = iOccupied;

//                 testX = map2D.scope.PointA.x + (double)(map2D.cells2D[cellIdx].key[0] + 0.5) * map2D.resolution;
//                 testY = map2D.scope.PointA.y + (double)(map2D.cells2D[cellIdx].key[1] + 0.5) * map2D.resolution;
//                 iCount3++;
//             }
//             else
//             {
//                 int id_x = (int)floor( (PtCoor.x() - map2D.scope.PointA.x + 0.5*map2D.resolution) * mapFactor );
//                 int id_y = (int)floor( (PtCoor.y() - map2D.scope.PointA.y + 0.5*map2D.resolution) * mapFactor );

//                 offset = (1 << (maxDepth - nodeDepth)) / 2;
//                 iCount4++;

//                 int cellIdx = 0;
//                 for (int iy = (id_y - offset); iy < (id_y + offset); iy++)
//                 {
//                     for (int ix = (id_x - offset); ix < (id_x + offset); ix++)
//                     {
//                         cellIdx = iy * max_val_x + ix;
//                         map2D.cells2D[cellIdx].iOccupied = iOccupied;

//                         testX = map2D.scope.PointA.x + (double)(map2D.cells2D[cellIdx].key[0] + 0.5) * map2D.resolution;
//                         testY = map2D.scope.PointA.y + (double)(map2D.cells2D[cellIdx].key[1] + 0.5) * map2D.resolution;

//                         iCount5++;
//                     }
//                 }
//             }

//             iNodeCnt[nodeDepth]++;
//         }
//         iCount++;
//     }

//     // MAP_Log_Level3("testX,%d,testY,%d",testX,testY);
//     std::cout << "testX," << testX << ",testY," << testY << std::endl;


//     outQuadTree("/media/wendy/DATA1/Work/AutoDrive/SLAMProject/MultiCol-SLAM_new/DataSet/mapData/quadTree.txt");
//     return true;
// }


// //get Map by input scope
// bool MapFactory::GetMapAll(Map2D_t& map2D)
// {
//     bool bRet = true;

//     double treeResolution = m_pQuadTree->getResolution();

//     Rectangle_t ValidScope;
//     m_pQuadTree->getMetricMinMax(ValidScope.PointA.x,ValidScope.PointA.y,ValidScope.PointB.x,ValidScope.PointB.y);

//     bRet = GetMapByScop(ValidScope, treeResolution, map2D);

//     return bRet;
// }

void MapFactory::outQuadTree(std::string outPath)
{
#if OPTION_OUTPUT_TXT_FLAG
    //QuadTree 并保存到TXT文件
    std::ofstream out( outPath );

    unsigned int maxDepth = m_pQuadTree->getTreeDepth();
    // double nodeSize = m_pQuadTree->getNodeSize( maxDepth );
    for (quadmap::QuadTree::iterator it = m_pQuadTree->begin(); it != m_pQuadTree->end(); ++it)
    {
        if(m_pQuadTree->isNodeOccupied(*it) && it.getDepth() <= maxDepth)
        {
            quadmap::point2d PtCoor = it.getCoordinate();

            out<< std::setiosflags(std::ios::fixed) << std::setprecision(7)
            << it.getDepth() << " "
            << PtCoor(0) << " "
            << 0.025 << " "
            << PtCoor(1) << std::endl;
        }
    }
    out.close();
    std::cout << "write quadmap point to txt finished!" << std::endl;
#endif //OPTION_OUTPUT_TXT_FLAG
}

//private function list------------
//when isExpandTree true, call this function
bool MapFactory::ExpandTree(Rectangle_t& cloudRect)
{
    // bool bRet(false);

    ExpandInfo_t expandInfo;
    getExpandInfo(cloudRect, expandInfo);

    //init new tree information
    // unsigned int oldDepth = m_pQuadTree->getTreeDepth();
    quadmap::QuadTree tree (m_resolution, m_pQuadTree->getTreeDepth() + 1, expandInfo.center);
    QuadNode* oldRoot = m_pQuadTree->getRoot();
    if(!oldRoot)
    {
        return false;
    }

    //decode:
    //div = 0: childID = (childID[0] & 0xF0) >> 4
    //div = 2: childID(j) = (childID_j & 0xF0) >> 4, old_subchildID(i) = ((childID_j>>2i) & 0x03)
    //div = 4: old subchildID i = 0,1,2,3; new subchildID(i) = ((childID[0]>>2i) & 0x03)
    std::vector<int> vPos;
    if ( expandInfo.iDiv == 0 )
    {
        int pos = (expandInfo.childID[0] & 0xF0) >> 4;
        vPos.push_back(pos);
        tree.addRootChildNode(oldRoot, vPos);
    }
    else if ( expandInfo.iDiv == 2 )
    {
        int posj[2];
        posj[0] = (expandInfo.childID[0] & 0xF0) >> 4;
        posj[1] = (expandInfo.childID[1] & 0xF0) >> 4;
        int oldtoNew = posj[1] - posj[0];

        int posi_new(-1),posi_old(-1);
        for (size_t j = 0; j < 2; j++)
        {
            for (size_t i = 0; i < 2; i++)
            {
                posi_old = ( (expandInfo.childID[j]>>(2*i) ) & 0x03 );
                posi_new = posi_old + oldtoNew;

                vPos.push_back(posj[j]);
                vPos.push_back(posi_new);

                //get old tree child node
                if (m_pQuadTree->nodeChildExists(oldRoot,posi_old))
                {
                    QuadNode* child = m_pQuadTree->getNodeChild(oldRoot,posi_old);
                    tree.addRootChildNode(child, vPos);
                }

                vPos.clear();
            }
            oldtoNew *= -1;
        }
        //end loop child node
    }
    else if ( expandInfo.iDiv == 4 )
    {
        // int posj[2];
        int posj(-1),posi_new(-1),posi_old(-1);

        int pos = (expandInfo.childID[0] & 0xF0) >> 4;
        vPos.push_back(pos);
        for (size_t i = 0; i < 4; i++)
        {
            posj = posi_old = i;
            posi_new = 3 - posi_old;

            vPos.push_back(posj);     //new childID
            vPos.push_back(posi_new); //new subchildID

            //get old tree child node
            if (m_pQuadTree->nodeChildExists(oldRoot,posi_old))
            {
                QuadNode* child = m_pQuadTree->getNodeChild(oldRoot,posi_old);
                tree.addRootChildNode(child, vPos);
            }
            vPos.clear();
        }
        //end loop child node
    }
    else
    {
        return false;
    }

    m_pQuadTree = new quadmap::QuadTree(tree);

    UpdateTreeRect(m_pQuadTree->GetTreeCenter(), m_pQuadTree->GetTreeWidth());

    return true;
}

void MapFactory::getExpandInfo(const Rectangle_t& cloudRect, ExpandInfo_t& expandInfo)
{
    int overflowNum = 0; //overflow sides
    uint8_t expandDir = RECT_EXP_MARK;
    if ( cloudRect.PointA.x < m_treeScope.PointA.x )
    {
        overflowNum++;
        expandDir &= RECT_EXP_LEFT;
    }
    if ( cloudRect.PointB.x > m_treeScope.PointB.x )
    {
        overflowNum++;
        expandDir &= RECT_EXP_RIGHT;
    }
    if ( cloudRect.PointA.y < m_treeScope.PointA.y )
    {
        overflowNum++;
        expandDir &= RECT_EXP_DOWN;
    }
    if ( cloudRect.PointB.y > m_treeScope.PointB.y )
    {
        overflowNum++;
        expandDir &= RECT_EXP_UP;
    }

    quadmap::point2d tree_center = m_pQuadTree->GetTreeCenter();
    double tree_width = m_pQuadTree->GetTreeWidth();

    if ( (overflowNum < 3) && (expandDir & 0x0F))
    {
        uint8_t tempDir = expandDir;
        vector<int> vDir;
        for (int i = 0; i < 4; i++)
        {
            if (tempDir & 0x01)
            {
                vDir.push_back(i);
            }
            tempDir >>= 1;
        }

        if ( 1 == overflowNum)
        {
            expandInfo.iDiv = 2;
            if (RECT_EXP_LEFT == expandDir)
            {
                expandInfo.center = quadmap::point2d(tree_center(0) - tree_width, tree_center(1));
                expandInfo.childID[0] = vDir[0] << 4;
                expandInfo.childID[0] += (1 << 2) + 0;
                expandInfo.childID[1] = vDir[1] << 4;
                expandInfo.childID[1] += (3 << 2) + 2;
            }
            else if (RECT_EXP_RIGHT == expandDir)
            {
                expandInfo.center = quadmap::point2d(tree_center(0) + tree_width, tree_center(1));
                expandInfo.childID[0] = vDir[0] << 4;
                expandInfo.childID[0] += (1 << 2) + 0;
                expandInfo.childID[1] = vDir[1] << 4;
                expandInfo.childID[1] += (3 << 2) + 2;
            }
            else if (RECT_EXP_DOWN == expandDir)
            {
                expandInfo.center = quadmap::point2d(tree_center(0), tree_center(1) - tree_width);
                expandInfo.childID[0] = vDir[0] << 4;
                expandInfo.childID[0] += (2 << 2) + 0;
                expandInfo.childID[1] = vDir[1] << 4;
                expandInfo.childID[1] += (3 << 2) + 1;
            }
            else if (RECT_EXP_UP == expandDir)
            {
                expandInfo.center = quadmap::point2d(tree_center(0), tree_center(1)+ tree_width);
                expandInfo.childID[0] = vDir[0] << 4;
                expandInfo.childID[0] += (2 << 2) + 0;
                expandInfo.childID[1] = vDir[1] << 4;
                expandInfo.childID[1] += (3 << 2) + 1;
            }
        }
        else //2 == overflowNum
        {
            expandInfo.iDiv = 0;
            if (RECT_EXP_LEFTDOWN == expandDir)
            {
                expandInfo.center = quadmap::point2d(tree_center(0) + tree_width, tree_center(1) + tree_width);
                expandInfo.childID[0] = vDir[0];
            }
            else if (RECT_EXP_RIGHTDOWN == expandDir)
            {
                expandInfo.center = quadmap::point2d(tree_center(0) - tree_width, tree_center(1) + tree_width);
                expandInfo.childID[0] = vDir[0];
            }
            else if (RECT_EXP_LEFTUP == expandDir)
            {
                expandInfo.center = quadmap::point2d(tree_center(0) + tree_width, tree_center(1) - tree_width);
                expandInfo.childID[0] = vDir[0];
            }
            else if (RECT_EXP_RIGHTUP == expandDir)
            {
                expandInfo.center = quadmap::point2d(tree_center(0) - tree_width, tree_center(1) - tree_width);
                expandInfo.childID[0] = vDir[0];
            }
        }
    }
    //if ( (overflowNum >= 3) || (expandDir & 0x0F) == 0x00 )
    else
    {
        expandInfo.iDiv = 4;
        expandInfo.center = quadmap::point2d(tree_center(0), tree_center(1));
        // expandInfo.childID[0] = 0x1B;
    }
}

void MapFactory::GenerateCubes()
{
    if (m_bIs3D)
    {
        GenerateOcTreeCube();
    }
    else
    {
        std::vector<cv::Vec3d> QuadPts;
        GetCubePoints(QuadPts);
        GeneratePtsCube(QuadPts);
    }
}

void MapFactory::DrawCubeMap()
{
    // draw delta occupied cells
    if (m_bIs3D)
    {
        drawOccupiedVoxels();
    }
    else
    {
        // draw delta occupied cells
        if (m_occupiedSize != 0) {
            glColor4f(0.2f, 0.7f, 1.0f, m_alphaOccupied);
            drawCubes(m_occupiedArray, m_occupiedSize);
        }
    }

    if (m_USUnoccupiedSize != 0) {
        glColor4f(0.2f, 0.0f, 0.2f, 0.1);
        drawCubes(m_USUnoccupiedArray, m_USUnoccupiedSize);
    }

}

void MapFactory::GenerateOcTreeCube()
{
    unsigned int cnt_occupied(0), cnt_occupied_thres(0);

    int itest1(0);
    unsigned int maxDepth = m_pOctoTree->getTreeDepth();
    octomap::OcTree::iterator it_bbx  = m_pOctoTree->begin(maxDepth);
    octomap::OcTree::iterator end_bbx = m_pOctoTree->end();
    for (; it_bbx != end_bbx; ++it_bbx)
    {
        if (m_pOctoTree->isNodeOccupied(*it_bbx)){ // occupied voxels
        //   if (m_pOctoTree->isNodeAtThreshold(*it_bbx)) ++cnt_occupied_thres;
        //   else                              ++cnt_occupied;
            ++cnt_occupied;
        }

        unsigned int nodeDepth = it_bbx.getDepth();
        if (nodeDepth > maxDepth )
        {
            itest1++;
        }
    }

    initGLArrays( cnt_occupied, m_occupiedSize, &m_occupiedArray);
    // initGLArrays( cnt_occupied_thres, m_occupiedThresSize, &m_occupiedThresArray);

    std::vector<octomath::Vector3> cube_template;
    initCubeTemplate(cube_template);

    unsigned int idx_occupied(0),idx_occupied_thres(0);
    octomap::OcTreeVolume voxel;
    it_bbx  = m_pOctoTree->begin(maxDepth);
    for (; it_bbx != end_bbx; ++it_bbx)
    {
        voxel = octomap::OcTreeVolume(it_bbx.getCoordinate(), it_bbx.getSize());

        if (m_pOctoTree->isNodeOccupied(*it_bbx)){ // occupied voxels
        //   if (m_pOctoTree->isNodeAtThreshold(*it_bbx)) {
        //     idx_occupied_thres = generateCube(voxel, cube_template, idx_occupied_thres, &m_occupiedThresArray);
        //   }
        //   else {
            idx_occupied = generateCube(voxel, cube_template, idx_occupied, &m_occupiedArray);
        //   }
        }
    }

}

bool MapFactory::GenerateQuadTreeCube(  quadmap::QuadTree* pQuadTree,
                                        unsigned int& glArraySize,
                                        GLfloat*** glArray)
{
    if (!pQuadTree /*|| glArray*/)
    {
        return false;
    }

    unsigned int cnt_occupied(0);

    //get image pixel occupied information
    unsigned int maxDepth = pQuadTree->getTreeDepth();
    unsigned int nodeDepth(0);
    quadmap::QuadTree::iterator it_bbx  = pQuadTree->begin(maxDepth);
    quadmap::QuadTree::iterator end_bbx = pQuadTree->end();
    for (; it_bbx != end_bbx; ++it_bbx)
    {
        if (pQuadTree->isNodeOccupied(*it_bbx)){
            // ++cnt_occupied;
            nodeDepth = it_bbx.getDepth();
            if ( nodeDepth == maxDepth )
            {
                ++cnt_occupied;
            }
            else if (nodeDepth < maxDepth)
            {
                int offset = (1 << (maxDepth - nodeDepth));
                cnt_occupied += offset*offset;
            }
        }
    }

    initGLArrays( cnt_occupied, glArraySize, glArray);
    // initGLArrays( cnt_occupied_thres, m_occupiedThresSize, &m_occupiedThresArray);

    std::vector<octomath::Vector3> cube_template;
    initCubeTemplate(cube_template);

    unsigned int idx_occupied(0);
    octomap::OcTreeVolume voxel;
    it_bbx  = pQuadTree->begin(maxDepth);
    int iCount(0),iCount1(0),iCount2(0),iCountN(0);
    for (; it_bbx != end_bbx; ++it_bbx)
    {
        if (pQuadTree->isNodeOccupied(*it_bbx)){ // occupied voxels

            quadmap::point2d QuadPts = it_bbx.getCoordinate();

            unsigned int nodeDepth = it_bbx.getDepth();
            if ( nodeDepth == maxDepth )
            {
                voxel = octomap::OcTreeVolume( octomap::point3d(QuadPts(0),QuadPts(1),0.5*m_resolution), m_resolution ); //TREE_RESOLUTION
                idx_occupied = generateCube(voxel, cube_template, idx_occupied, glArray);
                iCount1++;
            }
            else if (nodeDepth < maxDepth)
            {
                // double square = it_bbx.getSize();
                // int    uintCnt= round(square/m_resolution);
                int offset = (1 << (maxDepth - nodeDepth)) / 2;
                int cellIdx = 0;
                double testX(0.0),testY(0.0);
                for (int iy = -offset; iy < offset; iy++)
                {
                    for (int ix = -offset; ix < offset; ix++)
                    {
                        testX = QuadPts(0) + (double)(ix + 0.5) * m_resolution;
                        testY = QuadPts(1) + (double)(iy + 0.5) * m_resolution;

                        voxel = octomap::OcTreeVolume( octomap::point3d(testX,testY,0.5*m_resolution), m_resolution ); //TREE_RESOLUTION
                        idx_occupied = generateCube(voxel, cube_template, idx_occupied, glArray);

                        iCountN++;
                    }
                }
                iCount2++;
            }

            iCount++;
        }
    }

    return true;
}

void MapFactory::GeneratePtsCube(const std::vector<cv::Vec3d>& QuadPts)
{
    if (0 == QuadPts.size())
    {
        return;
    }

    initGLArrays(QuadPts.size(), m_occupiedSize, &m_occupiedArray);

    std::vector<octomath::Vector3> cube_template;
    initCubeTemplate(cube_template);

    unsigned int idx_occupied(0);
    octomap::OcTreeVolume voxel;
    for (int i = 0; i < QuadPts.size(); i++) {
        voxel = octomap::OcTreeVolume( octomap::point3d(QuadPts[i](0),QuadPts[i](1),QuadPts[i](2)), m_resolution ); //TREE_RESOLUTION
        idx_occupied = generateCube(voxel, cube_template, idx_occupied, &m_occupiedArray);
    }
}

//ultrasonic unoccupied voxel
void MapFactory::GenerateUSPtsCube(const std::vector<octomap::point3d>& USPts)
{
    if (0 == USPts.size())
    {
        return;
    }

    initGLArrays(USPts.size(), m_USUnoccupiedSize, &m_USUnoccupiedArray);

    std::vector<octomath::Vector3> cube_template;
    initCubeTemplate(cube_template);

    unsigned int idx_occupied(0);
    octomap::OcTreeVolume voxel;
    for (int i = 0; i < USPts.size(); i++) {
        voxel = octomap::OcTreeVolume( USPts[i], m_resolution ); //TREE_RESOLUTION
        idx_occupied = generateCube(voxel, cube_template, idx_occupied, &m_USUnoccupiedArray);
    }
}

//! add one cube to arrays
unsigned int MapFactory::generateCube(const octomap::OcTreeVolume& v,
                        const std::vector<octomath::Vector3>& cube_template,
                        const unsigned int& current_array_idx,
                        GLfloat*** glArray)
{
    // epsilon to be substracted from cube size so that neighboring planes don't overlap
    // seems to introduce strange artifacts nevertheless...
    double eps = 1e-5;

    octomath::Vector3 p;

    double half_cube_size = GLfloat(v.second /2.0 -eps);
    unsigned int i = current_array_idx;
    // Cube surfaces are in gl_array in order: back, front, top, down, left, right.
    // Arrays are filled in parallel (increasing i for all at once)
    // One color array for all surfaces is filled when requested

    p = v.first + cube_template[0] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[1] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[2] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[3] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[4] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[5] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i+= 3;  //-------------------

    p = v.first + cube_template[6] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[7] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[8] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[9] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[10] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[11] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i+= 3;  //-------------------

    p = v.first + cube_template[12] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[13] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[14] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[15] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[16] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[17] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i+= 3;  //-------------------

    p = v.first + cube_template[18] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[19] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[20] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[21] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[22] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[23] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i += 3;  //-------------------

    return i; // updated array idx
}

void MapFactory::initCubeTemplate(std::vector<octomath::Vector3>& cube_template)
{
    cube_template.clear();
    cube_template.reserve(24);

    cube_template.push_back(octomath::Vector3( 1, 1,-1));
    cube_template.push_back(octomath::Vector3( 1,-1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1,-1));
    cube_template.push_back(octomath::Vector3(-1, 1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1, 1));

    cube_template.push_back(octomath::Vector3(-1, 1,-1));
    cube_template.push_back(octomath::Vector3(-1,-1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1, 1));
    cube_template.push_back(octomath::Vector3(-1, 1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1,-1));
    cube_template.push_back(octomath::Vector3( 1,-1, 1));

    cube_template.push_back(octomath::Vector3(-1, 1, 1));
    cube_template.push_back(octomath::Vector3(-1,-1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1, 1));
    cube_template.push_back(octomath::Vector3(-1,-1, 1));
    cube_template.push_back(octomath::Vector3(-1,-1,-1));
    cube_template.push_back(octomath::Vector3(-1,-1, 1));

    cube_template.push_back(octomath::Vector3( 1, 1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1,-1));
    cube_template.push_back(octomath::Vector3(-1,-1,-1));
    cube_template.push_back(octomath::Vector3(-1, 1,-1));
    cube_template.push_back(octomath::Vector3(-1, 1, 1));
}

void MapFactory::initGLArrays(const unsigned int& num_cubes,
                                unsigned int& glArraySize,
                                GLfloat*** glArray)
{

    clearCubes(glArray, glArraySize);

    // store size of GL arrays for drawing
    glArraySize = num_cubes * 4 * 3;

    // allocate cube arrays, 6 quads per cube
    *glArray = new GLfloat* [6];
    for (unsigned i = 0; i<6; ++i){
        (*glArray)[i] = new GLfloat[glArraySize];
    }

}

void MapFactory::clearCubes(GLfloat*** glArray,
                                unsigned int& glArraySize)
{
    if (glArraySize != 0) {
        for (unsigned i = 0; i < 6; ++i) {
            delete[] (*glArray)[i];
        }
        delete[] *glArray;
        *glArray = NULL;
        glArraySize = 0;
    }
}

void MapFactory::drawOccupiedVoxels() const
{
     // draw binary occupied cells
    if (m_occupiedThresSize != 0) {
        glColor4f(0.0f, 0.0f, 1.0f, m_alphaOccupied);
        drawCubes(m_occupiedThresArray, m_occupiedThresSize);
    }

    // draw delta occupied cells
    if (m_occupiedSize != 0) {
        glColor4f(0.2f, 0.7f, 1.0f, m_alphaOccupied);
        drawCubes(m_occupiedArray, m_occupiedSize);
    }
}

void MapFactory::drawCubes(GLfloat** cubeArray, unsigned int cubeArraySize) const
{
    if (cubeArraySize == 0 || cubeArray == NULL){
    //   std::cerr << "Warning: GLfloat array to draw cubes appears to be empty, nothing drawn.\n";
        return;
    }

    // save current color
    // GLfloat* curcol = new GLfloat[4];
    // glGetFloatv(GL_CURRENT_COLOR, curcol);

    // cubeArraySize = 12;

    // top surfaces:
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertexPointer(3, GL_FLOAT, 0, cubeArray[0]);
    glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
    // bottom surfaces:
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertexPointer(3, GL_FLOAT, 0, cubeArray[1]);
    glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
    // right surfaces:
    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertexPointer(3, GL_FLOAT, 0, cubeArray[2]);
    glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
    // left surfaces:
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertexPointer(3, GL_FLOAT, 0, cubeArray[3]);
    glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
    // back surfaces:
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertexPointer(3, GL_FLOAT, 0, cubeArray[4]);
    glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
    // front surfaces:
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertexPointer(3, GL_FLOAT, 0, cubeArray[5]);
    glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);

    // reset color
    // glColor4fv(curcol);
    // delete[] curcol;
}
