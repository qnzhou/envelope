#pragma once

#include "SimplifierEC.h"
#include "Merger.h"
#include "VGrid.h"
#include "Stroke.h"
#include "LaplacianSolver.h"

#include "UIMesh3D.h"


enum _PoMesh  { pCurr = 0, pNormal, pOrig, pStr };


/////////////////////////////////////////////////////////////////////////////////////////
// CNPRAlg
// in order to run the algorithm in different thread
// mainframe posts requests as messages to this class 
//
/////////////////////////////////////////////////////////////////////////////////////////

class CNPRAlg : public CWinThread
{
	DECLARE_DYNCREATE(CNPRAlg)
    typedef List<Stroke> Strokes;
	typedef NPRMesh Mesh3D;
	//typedef GLUIMesh<Mesh3D> UIMesh3D;
	typedef NPRDrawer Drawer3D;
	typedef NPRHandler Handler3D;

    typedef std::map<std::pair<NPRVertex*, NPRVertex*>, Point3D> NormalMap;
    typedef std::map<std::pair<NPRVertex*, NPRVertex*>, real> DistMap;
    typedef Heap<real, NPREdge*, greater<real> > EdgeQueue;
    typedef std::set<NPRFace*> FaceSet;
	typedef std::map< NPREdge*, real > EdgeMap;
public:
	// User Message ////////////////////////////////////////////////////////
	enum _UM { 
		UM_OPEN_MESH = WM_USER + 1,
		UM_OPEN_LOG,
		UM_RUN,
        UM_NPR,
        UM_SNAPSHOT,
        UM_VISIBILITY,
        UM_CIRCUMVIEW,
        UM_RESET,
        UM_COMPUTE_EDGE_NORMAL,
        UM_COMPUTE_VERTEX_NORMAL,
        UM_SIMPLIFY,
        UM_MERGE,
        UM_SMOOTH,
        UM_OUTPUT,
        UM_OUTPUT_NILOY,
		UM_OUTPUT_VERSION4,
        UM_OPEN_EDGELIST,
        UM_SAVE_MESH,
        UM_LOAD_CORE_MESH,
        UM_SHOW_VOXELS,
        UM_SHOW_DEBUG,
        UM_COMPUTE_SOLID_VOXEL,
        UM_SHOW_EYE_POSITIONS,
        UM_GENERATE_VOXEL_SURFACE,
        UM_GENERATE_STROKES,
        UM_PROJECT_ONTO_VOXEL_HULL,
        UM_DEFORM_VOXEL_HULL,
        UM_LOAD_POINTS,
        UM_VOXELHULL_DEFORM_TO_MESH,
        UM_MAP_TO_ORI_MESH,
        UM_SNAP_TO_FEATURE_LINES,
        UM_FLIP_EDGE,
        UM_EXTRACT_NORMAL,
        UM_COMPUTE_LAPLACIAN_VECTOR,
        UM_REFINE_MESH,
        UM_FILTER_USE_NORMALS,
        UM_PARTITION,
        UM_SAVE_POINTS,
        UM_REMESH,
        UM_CUT_LONG_EDGE,
        UM_RECOVER_RIDGE,
		UM_COLLASPSE_EDGE,
		UM_COLLASPSE_THIN_TRIANGLES,
        UM_CLEAR,
        UM_DETECT_3_LOOP,
        UM_CORRECT_SELF_INTERSECTION,
        UM_SMOOTH_SEGMENT_NORMAL,
		UM_ADD_NOISE_VERTICES,
		UM_ADD_NOISE_FACES,
		UM_USE_BLEND,
		UM_SHOW_CREASES,
		UM_SMOOTH_VERTEX_NORMALS,
		UM_INIT_GL,
		UM_SPIN,
        UM_SAVE_COLOR,
        UM_LOAD_COLOR,
		UM_MDS,
        UM_GENERATE_ENVELOPE,
		UM_LOAD_ABSTRACTION_NET,
	};
	// Active Mesh /////////////////////////////////////////////////////////
	enum ActiveMesh { // mesh in focus
		meshA = 0,
		meshB,
        visualhull
	};

public:
	CNPRAlg();   
	virtual ~CNPRAlg();

public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();

	OSMutex& mutex() { return m_mutex; }
	
	GLUIClient3D& uiA() { return m_uiA; } 
	GLUIClient3D& uiB() { return m_uiB; }

	CString& path() {return m_path; }
	ActiveMesh& activeMesh() {return m_activeMesh; } 
    bool showVisibility() const { return m_drawerB.getShowVisibility(); }
    void setVisibilityMode(std::string mode) {
        if (mode[0] == 'F') {
            m_drawerA.setVisibilityMode(NPRDrawer::FACE);
            CONS("Face visibility.\r\n");
        } else if (mode[0] == 'E') {
            m_drawerA.setVisibilityMode(NPRDrawer::EDGE);
            CONS("Edge visibility.\r\n");
        } else if (mode[0] == 'N') {
            m_drawerA.setVisibilityMode(NPRDrawer::NONE);
        } else {
            CONS("Unknown visibility mode: %s\r\n", mode);
        }
    }
    void setMinTol(int tol) {
        m_drawerB.setMinTol(tol);
        NPRHandler::min_tol = tol;
    }
    void setMaxTol(int tol) { m_drawerB.setMaxTol(tol); }
    void setEyePos(int eye_pos) {
        if (eye_pos >= 0 && eye_pos < m_eye_positions.size()) {
            m_drawerA.setEyePosition(m_eye_positions[eye_pos]);
        } else {
            m_drawerA.setEyePosition(Point3D(0,0,1));
        }
        m_meshA.appearanceSensor().modify();
    }
    bool showVoxels() const { return m_drawerB.getShowVoxels(); }
    bool showDebug() const { return m_drawerB.getShowDebug(); }
    bool showEyePositions() const { return m_drawerB.getShowEyePositions(); }
    void setDisplayVisualHull(bool v);
    bool getDisplayVisualHull() const { return m_displayVisualHull; }
    void setRobustMode(bool v) { m_robustMode = v; }
    bool getRobustMode() const { return m_robustMode; }
    void setStrokeIndex(int index) { m_drawerB.setShowStroke(index); }
    void setSolverWeights(real w_laplacian, real w_source, real w_target) {
        LaplacianSolver::setWeights(w_laplacian, w_source, w_target);
    }
    bool getShowStrokes() const { return m_drawerB.getShowStrokes(); }
    void setShowStrokes(bool v) { m_drawerB.setShowStrokes(v); }
    bool getShowRefMesh() const { return m_drawerB.getShowRefMesh(); }
    void setShowRefMesh(bool v) { m_drawerB.setShowRefMesh(v); }
    bool getShowVertexNormal() const { return m_drawerB.getShowVertexNormal(); }
    void setShowVertexNormal(bool v) { m_drawerB.setShowVertexNormal(v); }
    bool getShowFaceNormal() const { return m_drawerB.getShowFaceNormal(); }
    void setShowFaceNormal(bool v) { m_drawerB.setShowFaceNormal(v); }
    bool getShowLaplacian() const { return m_drawerB.getShowLaplacian(); }
    void setShowLaplacian(bool v) { m_drawerB.setShowLaplacian(v); }

	void setCreaseThrehsold( float f ){	m_crease_threshold = f;	}
	real getCreaseThrehsold( ){	return m_crease_threshold;	}

	void setNoiseValue( float f ){	m_noise = f;	}
	real getNoiseValue( ){	return m_noise;	}


	void setStartIndex(int i){ m_start_index = i; }
	int getStartIndex(){ return m_start_index; }

	
	void setEndIndex(int i){ m_end_index = i; }
	int getEndIndex(){ return m_end_index; }

    void setAveDistTol(real v) { m_ave_dist_tol = v; }
    real getAveDistTol() const { return m_ave_dist_tol; }


    void setVoxResolution(int rex) { m_maxVoxelPerRow = rex; updateVoxelGrid(); }
    void setMinStrokeLen(real v) {
        foreach (si, m_strokes, Strokes) {
            if (si->length() < v)
                si->valid = false;
            else
                si->valid = true;
        }
        m_minStrokeLen = v;
    }
    void setAngleTol(real v) { m_angleTol = v; }
	void setVirtualAngleTol(real v) { m_virtualAngleTol = v; }
    void saveModelViewMatrix() {
        m_drawerA.saveModelViewMatrix(m_path);
        //m_drawerB.saveModelViewMatrix(m_path);
    }
    void loadModelViewMatrix() {
        m_drawerA.loadModelViewMatrix(m_path);
        m_meshA.appearanceSensor().modify();
        //m_drawerB.loadModelViewMatrix(m_path);
    }
    void setNormalLen(real v) {NPRDrawer::m_normalLength = v;}
    void setColor(real r, real g, real b) {
        NPRHandler::m_color.r() = r;
        NPRHandler::m_color.g() = g;
        NPRHandler::m_color.b() = b;
        NPRDrawer::m_color.r()  = r;
        NPRDrawer::m_color.g()  = g;
        NPRDrawer::m_color.b()  = b;
    }
    void setLighting(bool v) { NPRDrawer::m_lighting = v; }
    bool getLighting() const { return NPRDrawer::m_lighting; }
    void resampleAndMark(real dist) {
        Mesh3D& mesh = getActiveMesh();
        markSampledVertex(mesh, getAveFeatureEdgeDist(mesh) * dist);
    }
    void setAlpha(double alpha) { NPRDrawer::m_alpha = alpha; }
    void save_povray_edge() { 
        m_drawerA.dumpPovray_edge(m_path);
    }
    void save_povray_virtual_edge() {
        m_drawerA.dumpPovray_virtual_edge(m_path);
    }
    void save_povray_normal() {
        m_drawerA.dumpPovray_normal(m_path, false);
    }
    void save_povray_normal_colored() {
        m_drawerA.dumpPovray_normal(m_path, true);
    }
    void save_povray_mesh() {
        m_drawerA.dumpPovray_mesh(m_path);
    }
protected:
	// Message Map //////////////////////////////////////////////////////////
	DECLARE_MESSAGE_MAP()

	afx_msg void OnOpenMesh(WPARAM wParam, LPARAM lParam);
    afx_msg void OnSaveMesh(WPARAM wParam, LPARAM lParam);
	afx_msg void OnRun(WPARAM wParam, LPARAM lParam);
    afx_msg void OnNPR(WPARAM wParam, LPARAM lParam);
	afx_msg void OnShowCreases(WPARAM wParam, LPARAM lParam);
	afx_msg void OnBlend(WPARAM wParam, LPARAM lParam);
    afx_msg void OnSnapshot(WPARAM wParam, LPARAM lParam);
    afx_msg void OnVisibility(WPARAM wParam, LPARAM lParam);
    afx_msg void OnCircumView(WPARAM wParam, LPARAM lParam);
    afx_msg void OnReset(WPARAM wParam, LPARAM lParam);
    afx_msg void OnComputeEdgeNormal(WPARAM wParam, LPARAM lParam);
    afx_msg void OnComputeVertexNormal(WPARAM wParam, LPARAM lParam);
    afx_msg void OnSimplify(WPARAM wParam, LPARAM lParam);
    afx_msg void OnMerge(WPARAM wParam, LPARAM lParam);
    afx_msg void OnSmooth(WPARAM wParam, LPARAM lParam);
    afx_msg void OnOutput(WPARAM wParam, LPARAM lParam);
    afx_msg void OnOutputCoreMesh(WPARAM wParam, LPARAM lParam);
	afx_msg void OnOutputCoreMeshV4(WPARAM wParam, LPARAM lParam);
    afx_msg void OnOpenEdgeList(WPARAM wParam, LPARAM lParam);
    afx_msg void OnLoadCoreMesh(WPARAM wParam, LPARAM lParam);
    afx_msg void OnShowVoxels(WPARAM wParam, LPARAM lParam);
    afx_msg void OnShowDebug(WPARAM wParam, LPARAM lParam);
    afx_msg void OnComputeSolidVoxel(WPARAM wParam, LPARAM lParam);
    afx_msg void OnGenerateVoxelSurface(WPARAM wParam, LPARAM lParam);
    afx_msg void OnShowEyePositions(WPARAM wParam, LPARAM lParam);
    afx_msg void OnGenerateStrokes(WPARAM wParam, LPARAM lParam);
    afx_msg void OnProjectOntoVoxelHull(WPARAM wParam, LPARAM lParam);
    afx_msg void OnDeformVoxelHull(WPARAM wParam, LPARAM lParam);
    afx_msg void OnLoadPoints(WPARAM wParam, LPARAM lParam);
    afx_msg void OnSavePoints(WPARAM wParam, LPARAM lParam);
    afx_msg void OnVoxelHullDeformToMesh(WPARAM wParam, LPARAM lParam);
    afx_msg void OnMapToOriMesh(WPARAM wParam, LPARAM lParam);
    afx_msg void OnSnapToFeatureLines(WPARAM wParam, LPARAM lParam);
    afx_msg void OnFlipEdge(WPARAM wParam, LPARAM lParam);
	afx_msg void OnCollapseEdge(WPARAM wParam, LPARAM lParam);
    afx_msg void OnExtractNormal(WPARAM wParam, LPARAM lParam);
    afx_msg void OnComputeLaplacianVector(WPARAM wParam, LPARAM lParam);
    afx_msg void OnRefineMesh(WPARAM wParam, LPARAM lParam);
    afx_msg void OnFilterUseNormals(WPARAM wParam, LPARAM lParam);
    afx_msg void OnPartition(WPARAM wParam, LPARAM lParam);
    afx_msg void OnRemesh(WPARAM wParam, LPARAM lParam);
    afx_msg void OnCutLongEdge(WPARAM wParam, LPARAM lParam);
    afx_msg void OnRecoverRidge(WPARAM wParam, LPARAM lParam);
	afx_msg void OnCollapseThinTriangles(WPARAM wParam, LPARAM lParam);
	afx_msg void OnClear(WPARAM wParam, LPARAM lParam);
    afx_msg void OnDetect3Loop(WPARAM wParam, LPARAM lParam);
    afx_msg void OnCorrectSelfIntersection(WPARAM wParam, LPARAM lParam);
    afx_msg void OnSmoothSegmentNormal(WPARAM wParam, LPARAM lParam);
	afx_msg void OnAddNoiseToModelVertices(WPARAM wParam, LPARAM lParam);
	afx_msg void OnAddNoiseToModelFaces(WPARAM wParam, LPARAM lParam);
	afx_msg void OnSmoothVertexNormals(WPARAM wParam, LPARAM lParam);
	afx_msg void InitGL(WPARAM wParam, LPARAM lParam);
    afx_msg void OnSaveColor(WPARAM wParam, LPARAM lParam);
    afx_msg void OnLoadColor(WPARAM wParam, LPARAM lParam);
	afx_msg void OnSpin(WPARAM wParam, LPARAM lParam);
	afx_msg void OnMDS(WPARAM wParam, LPARAM lParam);
    afx_msg void OnGenerateEnvelope(WPARAM wParam, LPARAM lParam);
	afx_msg void OnLoadAbstractionNet(WPARAM wParam, LPARAM lParam);

private:
    void markNPREdges();
    bool IsVisible(const Point3D& viewDir, const Point3D& p) const;
    void createUniformEyePositions(int level);
    void resetEdge(NPRMesh& mesh);
    void resetFace(NPRMesh& mesh);
    void resetEyePosition();
    bool loadEdgeList(FILE* fid);
    bool loadEdgeListVn1(FILE* fid);
    bool loadEdgeListVn2(FILE* fid);
    void updateEdgeCount(const Array<EdgeListElement>& edgeList);
    void updateEdgeNormal(NPRMesh& mesh);
    void updateVertexNormal(NPRMesh& mesh);
    void removeIsolatedVertices(NPRMesh& mesh);
    bool loadData(const CString& filename, NPRMesh& mesh);
    bool loadCoreMeshV2(FILE* fid, NPRMesh& mesh);
    bool loadCoreMeshV3(FILE* fid, NPRMesh& mesh);
    bool loadCoreMeshV4(FILE* fid, NPRMesh& mesh);
    bool loadPoints(FILE* fid, NPRMesh& mesh);
    bool loadPointsV1(FILE* fid, NPRMesh& mesh);
    bool loadPointsV2(FILE* fid, NPRMesh& mesh);
    bool loadPointsV3(FILE* fid, NPRMesh& mesh);
    bool loadPointsV4(FILE* fid, NPRMesh& mesh);
    bool loadPointsV5(FILE* fid, NPRMesh& mesh);
    bool loadPointsV6(FILE* fid, NPRMesh& mesh);
    bool loadPointsDebug(FILE* fid, NPRMesh& mesh);
    bool savePointsV6(NPRMesh& mesh) const;
    void outputCoreMeshV2(NPRMesh& mesh) const;
    void outputCoreMeshV3(NPRMesh& mesh) const;
	void outputCoreMeshV4(NPRMesh& mesh) const;
    void outputCurveV1(NPRMesh& mesh) const;
    void outputCurveVn1(NPRMesh& mesh) const;
    void updateVoxelGrid();
    void generateVoxelSurface();
    void computeSolidVoxel(NPRMesh& mesh);
    void resolveDiagonalVoxels();
    bool resolveEdgeDiagonal(int i, int j, int k);
    bool resolveVertexDiagonal(int i, int j, int k);
    bool resolveEmptyDiagonal(int i, int j, int k);
    Stroke* traceStroke(NPREdge* edge);
    void viterbi(NPRMesh& mesh, Stroke* stroke);
    void viterbi(NPRMesh& mesh, SVertex::State* cst, SVertex* psv, bool debug=false);
    SVertex::State* maxState(SVertex* lsv, bool debug=false);
    void traceStates(SVertex::State* mst, bool debug=false);
    void computeLaplacianCoord(NPRMesh& mesh);
    void deform(NPRMesh& mesh);
    void deformNoAnchor(NPRMesh& mesh);
    void deformAnchor(NPRMesh& mesh);
    int interpolateStrokeOnVoxelShell();
    Stroke interpolateStroke(Stroke& stroke);
    void interpolateTo(Stroke& result, SVertex* sv, List<SVertex*>& temp);
    void smooth(NPRMesh* mesh, int iteration = 2);
    void dijkstraReset(NPRMesh* mesh);
    List<NPRVertex*> vertexDijkstraRun(NPRMesh& mesh, NPRVertex* source, NPRVertex* target, real radius = 1e3);
    List<NPRVertex*> vertexDijkstraAnchor(NPRVertex* source, real tol, Point3D dir);
    List<NPRVertex*> vertexDijkstraAnchor(NPRVertex* source, real tol, NPRVertex* neighbor = NULL);
    List<NPRVertex*> vertexDijkstraRun(NPRVertex* source, real radius);
    List<NPRFace*> faceDijkstraRun_old(NPRVertex* source, NPRVertex* target);
    List<NPRFace*> faceDijkstraRun(NPRMesh& mesh, NPRVertex* source, NPRVertex* target, real radius=1e3);
    List<NPRFace*> faceDijkstraRun(NPRFace* source, NPRFace* target);
    int partitionFaces(NPRVertex* source, real radius);
    void vHullToMesh();
    void copyMesh(NPRMesh* mesh1, NPRMesh* mesh2);
    NPRMesh& getActiveMesh();
    real computeAveMinDist(NPRMesh& mesh) const;
    void assignVertexIndex(NPRMesh& mesh);
    //void splitEdge(NPRMesh& mesh, NPREdge* edge, real ratio=0.5);
    NPRVertex* splitEdge(NPRMesh& mesh, NPREdge* edge, Point3D& p, bool approximate = true);
    void computeEdges(NPRMesh& mesh, real radius);
    NPRVertex* removeClosePoints(NPRMesh& mesh, NPRVertex* v, real radius);
    void computeAllPairGeodesicDist(NPRMesh& mesh);
    real geodesicDist(NPRMesh& mesh, NPRVertex* source, NPRVertex* target);
    List<NPRVertex*> vertexBellmanFord(NPRMesh& mesh, NPRVertex* source, NPRVertex* target);
    void resizeToUnitBox(NPRMesh& mesh);
    void flipZ(NPRMesh& mesh);
    real computeMeshMapping(NPRMesh& source, NPRMesh& target);
    real computeFastMeshMapping(NPRMesh& source, NPRMesh& target);
    real computeFastMeshMapping(Strokes& source, NPRMesh& target);
    NPRFace* getNearestPoint(NPRVertex* v, NPRMesh& target);
    NPRFace* getNearestPoint(Point3D& p, NPRMesh& target, Point3D& nearestPoint, NPRVertex* exception = NULL);
    NPRFace* getNearestPoint_slow(NPRVertex* v, NPRMesh& target);
    List<NPRFace*> getNearestPoints_slow(NPRVertex* v, NPRMesh& target);
    NPRFace* getNearestPoint(NPRVertex* v, FaceSet& faces);
    NPREdge* getNearestPoint_crease(NPRVertex* v, NPRMesh& target);
    Point3D getNearestPoint(Point3D& from1, Point3D& from2, Point3D& to1, Point3D& to2, int samples) const;
    real p2lDist(Point3D& p, Point3D& l1, Point3D& l2) const;
    void cutMesh(NPRMesh& mesh, Stroke& stroke, bool debug = false);
    void cutMesh2(NPRMesh& mesh, Stroke& storke, bool debug = false);
    List<NPRVertex*> cutThroughFaces(NPRMesh& mesh, List<NPRFace*>& f_list, Point3D& l1, Point3D& l2);
    NPREdge* getCommonEdge(NPRFace* f1, NPRFace* f2);
    NPRVertex* splitFace(NPRMesh& mesh, NPRFace* f, Point3D p, FaceSet& f_set);
    NPRVertex* splitFace(NPRMesh& mesh, NPRFace* f, Point3D p);
    int connectNearAnchors();
    int getNearAnchors(NPRVertex* v, NPRVertex*& neighbor);
    void trim(NPRMesh& mesh);
    bool trim(NPRVertex* v, NPRVertex* prev, real depth);
    bool trim(NPRVertex* v, real distLeft);
    bool insertIntoQ(EdgeQueue& Q, NPREdge* e);
    bool removeFromQ(EdgeQueue& Q, NPREdge* e);
    NPREdge* flipEdge(NPRMesh& mesh, NPREdge* e);
    void fixBadTriangles(NPRMesh& mesh, NPRVertex* v);
    void fixMesh(NPRMesh& mesh, bool preserveMode, bool preserveCrease=false, bool ifCollapseEdge = true, bool ifFlipEdge = true);
    List<NPRFace*> getRings(NPRVertex* v, int ring = 1);
    List<NPRVertex*> getNearByVertices(NPRMesh& mesh, NPRVertex* v, real radius);
    real gaussian(real x, real mu, real sig) const;
    bool onRidge(NPRVertex* v) const;
    void refineMesh(NPRMesh& mesh);
    void refineMesh2(NPRMesh& mesh);
    NPREdge* nextFeatureEdge(NPREdge* edge) const;
    NPREdge* prevFeatureEdge(NPREdge* edge) const;
    void smoothFeatureLineNormal(NPRMesh& mesh);
    void assignAnchorNormal(NPRMesh& mesh);
    int featureDegree(const NPRVertex* v, NPREdge::circulator& begin) const;
    void removeIsolatedAnchors(NPRMesh& mesh);
    void cutThinTriangles(NPRMesh& mesh, bool preserveMode=false);
    int collapseShortEdges(NPRMesh& mesh, real tol, bool preserveMode = true);
    int flipEdges(NPRMesh& mesh, bool preserveMode = true, bool preserveCrease = true);
    void registerNormals(NPRMesh& mesh);
    void resampleStrokes(NPRMesh& mesh);
    void maxErrorSampling(List<SVertex*>& sv_list,
        List<SVertex*>::iterator start,
        List<SVertex*>::iterator end,
        Array<bool>& keep, real err_tol);
    real pathDist(const List<NPRVertex*>& v_list) const;
    void filterAnchors(NPRMesh& mesh, real angle);
    void restoreAnchors(NPRMesh& mesh);
    bool isValidMesh(const NPRMesh& mesh) const;
    void computeDistToFeature(NPRMesh& mesh, bool coloring = false);
    List<NPRFace*> extractLocalMax(const NPRMesh& mesh) const;
    void partition(NPRMesh& mesh, List<NPRFace*>& seeds);
    Color3d getRandomColor() const;
    void resampleFeatureLines(NPRMesh& mesh, real sampleDist);
    real getAveFeatureEdgeDist(const NPRMesh& mesh) const;
    real getAveStrokeDist() const;
    void joinPieces(NPRMesh& mesh);
    bool remove_3_loop(NPRMesh& mesh, NPRVertex* v);
	bool smoothCheck(	Point3D newPosV1, Point3D newPosV2, Point3D newPosV3, 
						Point3D normalV1, Point3D normalV2, Point3D normalV3, real angle_tol);
    bool distCheck(NPRFace* face, real angle_tol);
    void cutLongEdges(NPRMesh& mesh, real dist_tol);
    void filterMisMappings_old(NPRMesh& mesh);
    void filterMisMappings(NPRMesh& mesh);
	bool isEdgeFlippable( NPREdge* e, NPRMesh& refMesh, double& diffDistance );
	bool neighborhoodNormalTest( Point3D newNormal, Point3D newNormal2, NPREdge* edge );
    void autoDetectVirtualEdges(NPRMesh& mesh);
    void scaleMappings(NPRMesh& mesh);
    void correctSelfIntersection(NPRFace* f1, NPRFace* f2);
    int break3Loops(NPRMesh& mesh, real tol);
    void eraseNeighborsFromQ(EdgeQueue& Q, NPRVertex* v);
    void cut(NPRMesh& mesh, NPREdge* e0, NPREdge* e1, NPREdge* e2);
    void smoothFeatureSegmentNormal(NPRMesh& mesh, real radius);
	NPREdge* getOppositeEdge( NPREdge* e, NPRMesh& mesh );
	void calculateCreaseAnglesForGeometryEdges(NPRMesh& mesh, real creaseThreshold, int pid = 0);
	void calculateCreaseAngles(NPRMesh& mesh, real creaseThreshold, int pid = 0);
    void markSampledVertex(NPRMesh& mesh, real dist);
    void subSample(std::vector<NPRVertex*>& vertices, int begin, int end, real len, real dist);
    void saveColorMap(FILE* fout, NPRMesh& mesh);
    void loadColorMap(FILE* fin, NPRMesh& mesh);
    void floodColor(NPRFace* f, const Color3d& color);

protected:

	// General //////////////////////////////////////////////////////////////////
	OSMutex m_mutex;
	ActiveMesh m_activeMesh;
	CString m_path;                        // model file name
    Array<Point3D> m_eye_positions;
    SphericalTriangle m_sphtri;
    SimplifierEC m_simplifier;
    Merger m_merger;
    NormalMap m_normalMap;
    DistMap m_distMap;
    VoxGrid m_vgrid;

	// A //////////////////////////////////////////////////////////////////////////
	Mesh3D m_meshA;                     // mesh
	UIMesh3D m_uiA;                      // GUI 
	Drawer3D m_drawerA;                // drawer
	Handler3D m_handlerA;               // handler

	// B //////////////////////////////////////////////////////////////////////////
	//Mesh3D m_meshB;
	UIMesh3D m_uiB;
    Drawer3D m_drawerB;
    Handler3D m_handlerB;

	// other //////////////////////////////////////////////////////////////////////
    bool m_displayVisualHull;
    Mesh3D m_vHull;
    Mesh3D m_points;
    Strokes m_strokes;
    real vox_size;

    static int m_maxVoxelPerRow;
    static bool m_robustMode; // might be slower
    static real m_tol;
    static real m_mapThreshold;
    static real m_ridgeAngle; // min dihidral angle for ridge
    static real m_minStrokeLen;
    static real m_visThreshold;
    static unsigned int m_seedCount; // for fast dijkstra algorithm
    static real m_angleTol; // used in normal-based filtering
	static real m_virtualAngleTol; // used to define virtual edges
    static real m_smooth_tol; // in degree
    static real m_dist_tol; // in degree
    static real m_ave_edge_len;
	static real m_crease_threshold;
	static real m_noise;
	static int m_start_index;
	static int m_end_index;
    static real m_ave_dist; // Average dist from voxel hull to mesh
    static real m_ave_dist_tol; // Stop criterion for enveloping
public:
	afx_msg void OnBlend();
	Drawer3D& returnFirstDrawer()
	{
		return	m_drawerA;  
	}
};


