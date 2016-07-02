// NPRAlg.cpp : implementation file
//

#include <ctime>
#include <queue>
#include "stdafx.h"
#include "Npr.h"
#include "NPRMesh.h"
#include "NPRDrawer.h"
#include "NPRHandler.h"
#include "NPRAlg.h"
#include "EdgeListElement.h"
#include "Mapper.h"
#include "CML/Calculus.h"
#include "TriTriIntersection.h"
#include "MACROS.h"
#include "CurvesFunctions.h"

/////////////////////////////////////////////////////////////////////////////////////////
// CNPRAlg
/////////////////////////////////////////////////////////////////////////////////////////

IMPLEMENT_DYNCREATE(CNPRAlg, CWinThread)

/////////////////////////////////////////////////////////////////////////////////////////
// Static member variables
/////////////////////////////////////////////////////////////////////////////////////////

int CNPRAlg::m_maxVoxelPerRow = 41;
bool CNPRAlg::m_robustMode = false;
real CNPRAlg::m_tol = 0.005;
real CNPRAlg::m_mapThreshold = 0.02;
real CNPRAlg::m_ridgeAngle = 60;
real CNPRAlg::m_minStrokeLen = 0;
real CNPRAlg::m_visThreshold = epsilon();
unsigned int CNPRAlg::m_seedCount = 1;
real CNPRAlg::m_angleTol = 30.0;
real CNPRAlg::m_smooth_tol = 25.0;
real CNPRAlg::m_dist_tol = 43.0;
real CNPRAlg::m_ave_edge_len = 0.0;
real CNPRAlg::m_virtualAngleTol = 0.0;// not use it by default.
real CNPRAlg::m_crease_threshold = 30;
real CNPRAlg::m_noise = 0.0;
int CNPRAlg::m_start_index = 0;
int CNPRAlg::m_end_index = 0;
real CNPRAlg::m_ave_dist = 1.0;
real CNPRAlg::m_ave_dist_tol = 0.003;
/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////
//extern "C" 
//{
//    #include "MDS\\ApplyMDS.h"
//}

#include "MDSlib.h"


CNPRAlg::CNPRAlg()
// General //
: m_activeMesh(meshA)
, m_meshA(2) // two sets of Points, one for coords another for normals
//, m_meshB(2)
, m_vHull(2)
, m_sphtri(Point3D(1.0,0.0,0.0), Point3D(0.0,1.0,0.0), Point3D(0.0,0.0,1.0))
, m_displayVisualHull(false)
{
	CriticalSection cs(m_mutex);

	// A ///////////////////////////////////////////////////////////////////
	m_uiA.setMesh(&m_meshA, &m_mutex); // mesh and lock for the user interface
	m_uiA.activatedCoords() = 0; // store coords in the first set (default)
	m_uiA.activatedNormals() = 1;  // store normals in the second
	m_uiA.setDrawer(&m_drawerA); // use this drawer instead of the default one
	m_uiA.setHandler(&m_handlerA); // when selection occurs functions of this handler should be evoked
	//m_handlerA.setSelectableVertices(true); // possible to select vertices
    //m_handlerA.setSelectableFaces(true); // possible to select faces
    m_uiA.wireframe = false;
    m_uiA.perspProj = true;

	// B ///////////////////////////////////////////////////////////////////
	//m_uiB.setMesh(&m_meshB, &m_mutex);
    // both display mesh A, so we will have two views.
	m_uiB.setMesh(&m_meshA, &m_mutex);
    m_uiB.setGrid(&m_vgrid);
	m_uiB.activatedNormals() = 1;
    m_drawerB.setSecondView(true);
    m_drawerB.setPointCloud(&m_points);
    m_uiB.setDrawer(&m_drawerB);
	m_uiB.setHandler(&m_handlerB); // when selection occurs functions of this handler should be evoked
	//m_handlerB.setSelectableVertices(true); // possible to select vertices
    //m_handlerB.setSelectableFaces(true); // possible to select faces
    m_handlerB.setSelectableEdges(true);
    m_handlerB.setSelectableVertices(true);
    m_handlerB.setMovableVertices(true);
    m_uiB.wireframe = false;
    m_uiB.perspProj = true;

    createUniformEyePositions(1);
    m_drawerB.setEyePositions(m_eye_positions);
    vox_size = 1.0;

}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

CNPRAlg::~CNPRAlg()
{
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

BOOL CNPRAlg::InitInstance()
{
	// TODO:  perform and per-thread initialization here
	return TRUE;
}

int CNPRAlg::ExitInstance()
{
	// TODO:  perform any per-thread cleanup here
	return CWinThread::ExitInstance();
}

BEGIN_MESSAGE_MAP(CNPRAlg, CWinThread)
	ON_THREAD_MESSAGE(UM_OPEN_MESH, OnOpenMesh)
    ON_THREAD_MESSAGE(UM_SAVE_MESH, OnSaveMesh)
	ON_THREAD_MESSAGE(UM_RUN, OnRun)
    ON_THREAD_MESSAGE(UM_NPR, OnNPR)
	//ON_THREAD_MESSAGE(ID_CREASE_BUTTON, OnBlend )
    ON_THREAD_MESSAGE(UM_SNAPSHOT, OnSnapshot)
    ON_THREAD_MESSAGE(UM_VISIBILITY, OnVisibility)
    ON_THREAD_MESSAGE(UM_CIRCUMVIEW, OnCircumView)
    ON_THREAD_MESSAGE(UM_RESET, OnReset)
    ON_THREAD_MESSAGE(UM_COMPUTE_EDGE_NORMAL, OnComputeEdgeNormal)
    ON_THREAD_MESSAGE(UM_COMPUTE_VERTEX_NORMAL, OnComputeVertexNormal)
    ON_THREAD_MESSAGE(UM_SIMPLIFY, OnSimplify)
    ON_THREAD_MESSAGE(UM_MERGE, OnMerge)
    ON_THREAD_MESSAGE(UM_SMOOTH, OnSmooth)
    ON_THREAD_MESSAGE(UM_OUTPUT, OnOutput)
    ON_THREAD_MESSAGE(UM_OUTPUT_NILOY, OnOutputCoreMesh)
	ON_THREAD_MESSAGE(UM_OUTPUT_VERSION4, OnOutputCoreMeshV4 )
	ON_THREAD_MESSAGE(UM_OPEN_EDGELIST, OnOpenEdgeList)
    ON_THREAD_MESSAGE(UM_LOAD_CORE_MESH, OnLoadCoreMesh)
    ON_THREAD_MESSAGE(UM_SHOW_VOXELS, OnShowVoxels)
    ON_THREAD_MESSAGE(UM_SHOW_DEBUG, OnShowDebug)
    ON_THREAD_MESSAGE(UM_COMPUTE_SOLID_VOXEL, OnComputeSolidVoxel)
    ON_THREAD_MESSAGE(UM_GENERATE_VOXEL_SURFACE, OnGenerateVoxelSurface)
    ON_THREAD_MESSAGE(UM_SHOW_EYE_POSITIONS, OnShowEyePositions)
    ON_THREAD_MESSAGE(UM_GENERATE_STROKES, OnGenerateStrokes)
    ON_THREAD_MESSAGE(UM_PROJECT_ONTO_VOXEL_HULL, OnProjectOntoVoxelHull)
    ON_THREAD_MESSAGE(UM_DEFORM_VOXEL_HULL, OnDeformVoxelHull)
    ON_THREAD_MESSAGE(UM_LOAD_POINTS, OnLoadPoints)
    ON_THREAD_MESSAGE(UM_VOXELHULL_DEFORM_TO_MESH, OnVoxelHullDeformToMesh)
    ON_THREAD_MESSAGE(UM_MAP_TO_ORI_MESH, OnMapToOriMesh)
    ON_THREAD_MESSAGE(UM_SNAP_TO_FEATURE_LINES, OnSnapToFeatureLines)
    ON_THREAD_MESSAGE(UM_EXTRACT_NORMAL, OnExtractNormal)
    ON_THREAD_MESSAGE(UM_COMPUTE_LAPLACIAN_VECTOR, OnComputeLaplacianVector)
    ON_THREAD_MESSAGE(UM_REFINE_MESH, OnRefineMesh)
    ON_THREAD_MESSAGE(UM_FILTER_USE_NORMALS, OnFilterUseNormals)
    ON_THREAD_MESSAGE(UM_PARTITION, OnPartition)
    ON_THREAD_MESSAGE(UM_SAVE_POINTS, OnSavePoints)
    ON_THREAD_MESSAGE(UM_REMESH, OnRemesh)
    ON_THREAD_MESSAGE(UM_CUT_LONG_EDGE, OnCutLongEdge)
    ON_THREAD_MESSAGE(UM_RECOVER_RIDGE, OnRecoverRidge)
	ON_THREAD_MESSAGE(UM_FLIP_EDGE, OnFlipEdge)
    ON_THREAD_MESSAGE(UM_COLLASPSE_EDGE, OnCollapseEdge)
	ON_THREAD_MESSAGE(UM_COLLASPSE_THIN_TRIANGLES, OnCollapseThinTriangles)
    ON_THREAD_MESSAGE(UM_CLEAR, OnClear)
    ON_THREAD_MESSAGE(UM_DETECT_3_LOOP, OnDetect3Loop)
    ON_THREAD_MESSAGE(UM_CORRECT_SELF_INTERSECTION, OnCorrectSelfIntersection)
	ON_THREAD_MESSAGE(UM_SMOOTH_SEGMENT_NORMAL, OnSmoothSegmentNormal)
	ON_THREAD_MESSAGE(UM_ADD_NOISE_VERTICES, OnAddNoiseToModelVertices )
	ON_THREAD_MESSAGE(UM_ADD_NOISE_FACES, OnAddNoiseToModelFaces )	
	ON_THREAD_MESSAGE(UM_USE_BLEND, OnBlend)
	ON_THREAD_MESSAGE(UM_SHOW_CREASES, OnShowCreases)
	ON_THREAD_MESSAGE(UM_SMOOTH_VERTEX_NORMALS, OnSmoothVertexNormals )
	ON_THREAD_MESSAGE(UM_INIT_GL, InitGL )
	ON_THREAD_MESSAGE(UM_SPIN, OnSpin )	
    ON_THREAD_MESSAGE(UM_SAVE_COLOR, OnSaveColor)
    ON_THREAD_MESSAGE(UM_LOAD_COLOR, OnLoadColor)
	ON_THREAD_MESSAGE(UM_LOAD_ABSTRACTION_NET, OnLoadAbstractionNet)
	ON_THREAD_MESSAGE(UM_MDS, OnMDS)
	ON_THREAD_MESSAGE(UM_GENERATE_ENVELOPE, OnGenerateEnvelope)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////////////////
// CNPRAlg message handlers
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnOpenMesh(WPARAM wParam, LPARAM lParam)
{
	CriticalSection cs(m_mutex);

	//Mesh3D& mesh = (m_activeMesh == meshA) ? m_meshA : m_meshB;

    Mesh3D& mesh = getActiveMesh();

	if (!loadVRML(m_path, mesh))
		return;

    if (m_activeMesh != visualhull) {
	    //mesh.remStandAloneVer();
        Point3D min_mesh, max_mesh;
        mesh.getBoundingBox(min_mesh, max_mesh);
        translate(mesh, (min_mesh*0.5+max_mesh*0.5)*(-1), 0);
        //translateCenter(mesh, 0);
        resizeToUnitBox(mesh);
    }
    /*
    double factor = 1;
	foreach(vi, m_meshA.vertices(), Mesh3D::Vertices) {
        if (abs(vi->p().x()) > factor) { factor = abs(vi->p().x()); }
        if (abs(vi->p().y()) > factor) { factor = abs(vi->p().y()); }
        if (abs(vi->p().z()) > factor) { factor = abs(vi->p().z()); }
	}
    scale(m_meshA, 1.0/factor);
    */
    //scale(m_meshA, 0.008);
	generateNormals(mesh, 0, 1);// use coords in 0 and store normals in 1

    assignVertexIndex(mesh);

    if (m_activeMesh != visualhull) {
        updateVoxelGrid();
        m_drawerB.setVoxelGrid(&m_vgrid);
        m_drawerB.setReferenceMesh(&m_vHull);

        m_strokes.clear();
    } else {
        m_ave_edge_len = mesh.getAveEdgeLength();
        CONS("Average edge length for voxel hull is %f.\r\n", m_ave_edge_len);
    }

	mesh.eraseNotConnectedVertices();
    mesh.computeFaceAreas(0);
    mesh.computeFaceNormals(0);
    mesh.computeVertexNormals(0);
 
	mesh.clearSensor().modify();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnSaveMesh(WPARAM wParam, LPARAM lParam)
{
    CriticalSection cs(m_mutex);

    Mesh3D& mesh = getActiveMesh();

	if (!saveVRML(m_path, mesh))
		return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnRun(WPARAM wParam, LPARAM lParam)
{
	Mesh3D& mesh = getActiveMesh();
	srand( (unsigned)time( NULL ) );
	mesh.colorCharts(m_drawerB.getMinTol());

	m_drawerA.setShowBorder( !m_drawerA.getShowBorder() ); 
    /*
	int i = 0;

	//for (int i=0; i < m_meshA.size(); ++i) {
	//	m_meshB[i]->p(1) = m_meshA[i]->p(1);
	//}
	foreach(fi, mesh.faces(), NPRMesh::Faces) {
		if (++i%100 == 0) {
			mesh.appearanceSensor().modify();
			// First, do not go to sleep while you are holding the lock (m_mutex)
			// Second, do not go to sleep too often (Windows context switches are expensive)
			//Third, unless you are changing mesh connectivity don’t even bother to lock 
			CONS("HELLO WORLD! == %d\r\n", i);
			Sleep(1);
		}
		
		CriticalSection cs(m_mutex);
		NPRFace* f = &*fi;
		f->color().r() = rand();
		f->color().g() = rand();		
		f->color().b() = rand();

		f->color() /= RAND_MAX;
	}
    */
	mesh.appearanceSensor().modify();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnNPR(WPARAM wParam, LPARAM lParam) {

	NPRMesh* mesh = &(getActiveMesh());

	foreach (ei, mesh->edges(), NPRMesh::Edges) 
	{
		NPRMesh::Edge* e = &(*ei);
		e->eCreaseAngle = 0;
		e->eCrease = false;
	}

	for( int i=0; i<10; i++ )
		OnSmoothVertexNormals(wParam, lParam);
	
	if( !m_drawerA.drawNPRShader )
	{
		CONS("Crease Threhsold %f \r\n", 0.0 );
		NPRMesh& mesh = getActiveMesh();

		calculateCreaseAnglesForGeometryEdges( mesh, 0.0 );
		generateNormals(mesh, 0, 1);
		generateNormalsCreases(mesh, 0);

		for( int i=0; i<10; i++ )
			OnSmoothVertexNormals(wParam, lParam);

		CONS("Done\r\n");
	}
	m_drawerA.drawNPRShader = !m_drawerA.drawNPRShader;

}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnShowCreases(WPARAM wParam, LPARAM lParam) {
   
	NPRMesh* mesh = &(getActiveMesh());

	foreach (ei, mesh->edges(), NPRMesh::Edges) 
	{
		NPRMesh::Edge* e = &(*ei);
		e->eCreaseAngle = 0;
		e->eCrease = false;
	}

	if( !m_drawerA.drawCreaseShader )
	{
		CONS("Crease Threhsold %f \r\n", getCreaseThrehsold() );
		NPRMesh& mesh = getActiveMesh();

		calculateCreaseAngles( mesh, getCreaseThrehsold() );
		generateNormals(mesh, 0, 1);
		generateNormalsCreases(mesh, 0);

		CONS("Done\r\n");
	}
	m_drawerA.drawCreaseShader = !m_drawerA.drawCreaseShader;

}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnSmoothVertexNormals(WPARAM wParam, LPARAM lParam) {
	
	const double alpha = 0.6;
	const int nid = 1;
	NPRMesh* mesh = &(getActiveMesh());

	for( int itr = 0; itr<5; itr++ )
	{
		// set normals for vertices
		foreach (vi, mesh->vertices(), NPRMesh::Vertices) {
			if (!vi->connected())
				continue;

			bool ifInside = false;
			Point3D N(0,0,0);
			NPRMesh::Edge::circulator ec = vi->edge_circulator();
			do {
				Point3D normal = (*ec)->t()->p(nid);

				if( normal.abs() >= 0.9 && normal.abs() <= 1.1 )
				{
					N +=   normal.unit(); 
					ifInside = true;
				}
			} while (++ec != vi->edge_circulator() && ec != NULL);

			if( ifInside )
				vi->p(nid) = vi->p(nid)*alpha + N.unit() * (1-alpha);
			
			vi->p(nid).normalize();
		}
	}
}


void CNPRAlg::InitGL(WPARAM wParam, LPARAM lParam)
{

	//if (client() == 0 || !client()->valid()) return;
	//client()->initGL();
	glEnable(GL_POLYGON_OFFSET_LINE);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glEnable(GL_POLYGON_OFFSET_POINT);
	glEnable(GL_DEPTH_TEST);
#ifndef PRINT_DRAW
	glEnable(GL_BLEND);
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	//glBlendFunc(GL_SRC_ALPHA ,GL_ONE);
	glEnable(GL_LINE_SMOOTH);
#endif
	glEnable(GL_POINT_SMOOTH);

	// Light ////////////////////////////////////////////////////////////////////////
	glShadeModel(GL_SMOOTH);
	// Turn on OpenGL lighting
	glEnable(GL_LIGHTING);


//#ifdef PRINT_DRAW
	GLfloat Ambient[]  = { 0.0f, 0.0f, 0.0f, 1.0f};
	Color3d dif3f = color; //(client())? client()->diffuseColor : Color3f(1.0f,1.0f,1.0f);
	GLfloat Diffuse[]  = { DIFFUSE_FACTOR*dif3f.r() , DIFFUSE_FACTOR*dif3f.g(), DIFFUSE_FACTOR*dif3f.b(), 1.0f}; 
	

	// first light
	glLightfv(GL_LIGHT0, GL_AMBIENT, Ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, Diffuse);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, Specular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPos0);
	glEnable(GL_LIGHT0);


 
	// second light
	glLightfv(GL_LIGHT1, GL_AMBIENT, Ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, Diffuse);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, Specular);
	glLightfv(GL_LIGHT1, GL_POSITION, LightPos1);
	glEnable(GL_LIGHT1);



 
	// third light
	glLightfv(GL_LIGHT2, GL_AMBIENT, Ambient);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, Diffuse);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, Specular);
	glLightfv(GL_LIGHT2, GL_POSITION, LightPos2);
	glEnable(GL_LIGHT2);



	// fourth light
	GLfloat Diffuse4[]  = { 0.5*dif3f.r() , 0.5*dif3f.g(), 0.5*dif3f.b(), 1.0f}; // for others
	glLightfv(GL_LIGHT3, GL_AMBIENT, Ambient);
	glLightfv(GL_LIGHT3, GL_DIFFUSE, Diffuse4);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, Specular);
	glLightfv(GL_LIGHT3, GL_POSITION, LightPos3);

	if( use_bottom_light )
		glEnable(GL_LIGHT3);
 


	GLfloat Specular[] = { 0.01f, 0.01f, 0.01f, 1.0f};
	GLfloat Shine = 30.0f;
	
	GLfloat Dif[]  = { 1.0f, 0.0f, 0.0f, 1.0f};
	//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, Dif);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, Specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, &Shine);

	glEnable(GL_COLOR_MATERIAL);
	//glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	//glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
}





void CNPRAlg::OnBlend(WPARAM wParam, LPARAM lParam) {
	m_drawerA.drawBlend = !m_drawerA.drawBlend;

	if( m_drawerA.drawBlend )
		CONS("Blend enabled \r\n");
	else
		CONS("Blend disabled \r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnSnapshot(WPARAM wParam, LPARAM lParam) {
    Mesh3D* m_mesh = &(getActiveMesh());

    markNPREdges();
    switch(m_drawerA.getVisibilityMode()) {
        case NPRDrawer::FACE:
        {
            /*
            Point3D z = m_drawerA.getViewDir();
            int pid = 0;
            foreach (ei, m_mesh->edges(), NPRMesh::Edges) {
                if (ei->isProp(EP_NPREDGE)) {
                    //Point3D n0 = triangleFaceNormal(ei->face(), pid);
                    //Point3D n1;
                    //if (ei->twin() != 0) {
                    //    n1 = triangleFaceNormal(ei->twin()->face(), pid);
                    //} else { n1 = n0; }
                    //if (n0*z < 0 || n1*z < 0) {
                        // Check for visibility
                        if (ei->isInterior()) {
                            if (!ei->face()->isVisible() &&
                                !ei->twin()->face()->isVisible()) {
                                    ei->remProp(EP_NPREDGE);
                                //ei->nprEdge = false;
                            }
                        } else {
                            if (!ei->face()->isVisible()) {
                                ei->remProp(EP_NPREDGE);
                                //ei->nprEdge = false;
                            }
                        }
                    //} else {
                    //    ei->nprEdge = false;
                    //}
                }
                if (ei->isProp(EP_NPREDGE)) { ei->count++; }
            }
            */
            foreach (fi, m_mesh->faces(), NPRMesh::Faces) {
                if (fi->isVisible()) {
                    fi->m_visCount++;
                }
            }
            break;
        }
        case NPRDrawer::EDGE:
        {
            foreach (ei, m_mesh->edges(), NPRMesh::Edges) {
                ASSERT(ei->visible_len >= 0);
                if (ei->isProp(EP_NPREDGE)) {
                    //ei->count += ei->visible_len;
                    if (ei->visible_len > 2) {
                        ei->count++;
                    }
                }
            }
            break;
        }
        default:
            CONS("Visibility mode not set.");
    }
    m_mesh->appearanceSensor().modify();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnVisibility(WPARAM wParam, LPARAM lParam) {
    m_drawerB.setShowVisibility(!m_drawerB.getShowVisibility());
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////


void CNPRAlg::OnCircumView(WPARAM wParam, LPARAM lParam) {
    if (m_drawerA.getVisibilityMode() == NPRDrawer::NONE) {
        CONS("ERROR: please pick a visibility mode.\r\n");
        return;
    }

    NPRMesh& mesh = getActiveMesh();
    resetEdge(mesh);
    resetFace(mesh);

    int num_shots = 0;

    Point3D ori_eye_pos = m_drawerA.getEyePosition();
    CONS("Default eye position: %f %f %f\r\n",
        ori_eye_pos.x(), ori_eye_pos.y(), ori_eye_pos.z());
    CONS("%d Eye positions\r\n", m_eye_positions.size());
    foreach_const(i, m_eye_positions, Array<Point3D>) {
        //if (i->y() < 1.0) { continue; }
        //if (i->y() > 1.0 || i->y() < 0.0) { continue; }
        while (m_mutex.tryLock() == false);
        m_drawerA.setEyePosition(*i);
        m_drawerA.setRedrawDone(false);
        m_meshA.appearanceSensor().modify();
        //m_meshB.appearanceSensor().modify();
        do {
            m_mutex.unlock();
            //Sleep(1000);
            Yield(); // yield so the drawer could update the visiblity info.
            while (m_mutex.tryLock() == false);
        } while (m_drawerA.getRedrawDone() == false);
        OnSnapshot(0,0);
        CONS("Shot %d: %f %f %f\r\n", num_shots,
            i->x(), i->y(), i->z());
        num_shots++;
        m_mutex.unlock();
    }
    m_drawerA.setEyePosition(ori_eye_pos);

    int max_count = 0;
    int min_count = m_eye_positions.size();
    double average = 0.0;

    switch(m_drawerA.getVisibilityMode()) {
        case NPRDrawer::FACE:
            foreach (fi, mesh.faces(), NPRMesh::Faces) {
                average += fi->m_visCount;
                if (fi->m_visCount > max_count) {
                    max_count = fi->m_visCount;
                }
                if (fi->m_visCount < min_count) {
                    min_count  = fi->m_visCount;
                }
            }
            average /= double(mesh.faces().size());
            break;
        case NPRDrawer::EDGE:
            foreach (ei, mesh.edges(), NPRMesh::Edges) {
                average += ei->count;
                if (ei->count > max_count) {
                    max_count = ei->count;
                }
                if (ei->count < min_count) {
                    min_count = ei->count;
                }
            }
            average /= double(mesh.edges().size());
            break;
    }

    CONS("max=%d  min=%d  ave=%f\r\n", max_count, min_count, average);
    CONS("CircumView Done! %d shots\r\n", num_shots);
    m_drawerB.setMinTol(min_count);
    m_drawerB.setMaxTol(max_count);

    switch(m_drawerA.getVisibilityMode()) {
        case NPRDrawer::FACE:
            foreach (fi, mesh.faces(), NPRMesh::Faces) {
                fi->m_visRatio = double(fi->m_visCount-min_count)/double(max_count-min_count);
            }
            //m_visThreshold = 1/double(max_count-min_count);
            break;
        case NPRDrawer::EDGE:
            break;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnReset(WPARAM wParam, LPARAM lParam) {
    NPRMesh& mesh = getActiveMesh();
    resetEdge(mesh);
    resetFace(mesh);
    resetEyePosition();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnComputeEdgeNormal(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();

    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        Point3D normal = triangleFaceNormal(ei->face(), 0);
        if (normal.abs() < epsilon()) {
            CONS("WRANING: degenerated normal (%f %f %f)!\r\n",
                normal.x(), normal.y(), normal.z());
        }
        normal.normalize();
        m_normalMap[std::pair<NPRVertex*, NPRVertex*>(ei->s(), ei->t())] = normal;
        if (!ei->twin()) {
            m_normalMap[std::pair<NPRVertex*, NPRVertex*>(ei->t(), ei->s())] = normal;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnComputeVertexNormal(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
	
	foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        vi->normal = Point3D(0.0, 0.0, 0.0);
        NPREdge::circulator ci = vi->edge_circulator();
        do {
            vi->normal += triangleFaceUnitNormal((*ci)->face(), 0);
        } while (++ci != vi->edge_circulator() && ci != 0);
        vi->normal.normalize();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnSimplify(WPARAM wParam, LPARAM lParam) 
{
    NPRMesh& mesh = getActiveMesh();

    typedef Array<std::pair<NPRVertex*, NPRVertex*> > EdgeArray;
    real sampleLen = getAveFeatureEdgeDist(mesh)*2.0/3.0;
    //CriticalSection cs(m_mutex);
    m_mutex.lock();
    EdgeArray fixed_edges;
    Array<int> edge_count;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        if (ei->count > m_drawerB.getMinTol()) {
            ei->s()->addProp(VP_ANCHOR);
            ei->t()->addProp(VP_ANCHOR);
            fixed_edges.push_back(std::pair<NPRVertex*, NPRVertex*>(ei->s(), ei->t()));
            edge_count.push_back(ei->count);
        }
    }
    ASSERT(fixed_edges.size() == edge_count.size());
    m_simplifier.set(&mesh, 0);
    m_simplifier.simplify();

    int index = 0;
    foreach (pi, fixed_edges, EdgeArray) {
        NPREdge* ei = pi->first->getEdgeTo(pi->second);
        ei->count = max(ei->count, edge_count[index]);
        ei->twin()->count = max(ei->twin()->count,edge_count[index]);
        index++;
    }

    removeIsolatedVertices(mesh);
    fixMesh(mesh, true);
    m_mutex.unlock();
    resampleFeatureLines(mesh, sampleLen);
    removeIsolatedVertices(mesh);
    removeIsolatedAnchors(mesh);
    restoreAnchors(mesh);
    //updateEdgeNormal(mesh);
    //updateVertexNormal(mesh);
    //mesh.appearanceSensor().modify();
    generateNormals(mesh, 0, 1);// use coords in 0 and store normals in 1
    CONS("Simplification Done!\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnMerge(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    mesh.computeFaceAreas(0);
    mesh.computeFaceNormals(0);

    typedef Array<std::pair<NPRVertex*, NPRVertex*> > EdgeArray;
    CriticalSection cs(m_mutex);
    EdgeArray fixed_edges;
    Array<int> edge_count;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        if (ei->count > m_drawerB.getMinTol()) {
            ei->s()->addProp(VP_ANCHOR);
            ei->t()->addProp(VP_ANCHOR);
            fixed_edges.push_back(std::pair<NPRVertex*, NPRVertex*>(ei->s(), ei->t()));
            edge_count.push_back(ei->count);
        }
    }
    ASSERT(fixed_edges.size() == edge_count.size());
    m_merger.set(&mesh, 0);
    m_merger.merge();

    // make all edge feature lines for normal computation.
    /*
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        if (ei->count == 0)
            ei->count = 1;
        else
            ei->count = 180;
    }
    */
    int index = 0;
    foreach (pi, fixed_edges, EdgeArray) {
        NPREdge* ei = pi->first->getEdgeTo(pi->second);
        ei->count = max(ei->count, edge_count[index]);
        index++;
    }

    removeIsolatedVertices(mesh);
    updateEdgeNormal(mesh);
    assignAnchorNormal(mesh);

    /*
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        if (ei->count == 1)
            ei->count = 0;
    }
    */

    //updateVertexNormal(mesh);
    autoDetectVirtualEdges(mesh);
    // Mark virtual edges.
    foreach (pi, fixed_edges, EdgeArray) {
        NPREdge* ei = pi->first->getEdgeTo(pi->second);
        if (ei->count < m_drawerB.getMaxTol()) {
            ei->count = 0;
            ei->twin()->count = 0;
        }
    }
    mesh.appearanceSensor().modify();
    CONS("Merging Done!\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnSmooth(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        if (ei->count > m_drawerB.getMinTol()) {
            ei->s()->addProp(VP_ANCHOR);
            ei->t()->addProp(VP_ANCHOR);
        }
    }

    smooth(&mesh);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnOutput(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    restoreAnchors(mesh);

    outputCurveVn1(mesh);

    CONS("Edges output done.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnOutputCoreMesh(WPARAM wParam, LPARAM lParam)
{
    outputCoreMeshV3(getActiveMesh());
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnOutputCoreMeshV4(WPARAM wParam, LPARAM lParam)
{
    outputCoreMeshV4(getActiveMesh());
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnOpenEdgeList(WPARAM wParam, LPARAM lParam)
{
	CriticalSection cs(m_mutex);
    Mesh3D& mesh = getActiveMesh();
    if (!loadData(m_path, mesh)) {
        CONS("ERROR: failed to load core mesh from %s.\r\n", m_path);
        return;
    }
    for (int i=0; i<10; i++) {
        smoothFeatureLineNormal(mesh);
    }
    assignAnchorNormal(mesh);
    registerNormals(mesh);
    return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnLoadCoreMesh(WPARAM wParam, LPARAM lParam)
{
	CriticalSection cs(m_mutex);

	//Mesh3D& mesh = (m_activeMesh == meshA) ? m_meshA : m_meshB;
    NPRMesh& mesh = getActiveMesh();
    if (!loadData(m_path, mesh)) {
        CONS("ERROR: failed to load core mesh from %s.\r\n", m_path);
        return;
    }
	generateNormals(mesh, 0, 1);// use coords in 0 and store normals in 1
	mesh.clearSensor().modify();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnShowVoxels(WPARAM wParam, LPARAM lParam)
{
    m_drawerB.setShowVoxels(!m_drawerB.getShowVoxels());
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnShowDebug(WPARAM wParam, LPARAM lParam)
{
    m_drawerB.setShowDebug(!m_drawerB.getShowDebug());
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnComputeSolidVoxel(WPARAM wParam, LPARAM lParam)
{
	CriticalSection cs(m_mutex);
    NPRMesh& mesh = getActiveMesh();

    computeSolidVoxel(mesh);

    resolveDiagonalVoxels();
    CONS("Done computing solid voxels.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnGenerateVoxelSurface(WPARAM wParam, LPARAM lParam)
{
    generateVoxelSurface();
    CONS("Done generating voxel faces (%d vertices).\r\n", m_vHull.nVertices());
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnShowEyePositions(WPARAM wParam, LPARAM lParam)
{
    m_drawerB.setShowEyePositions(!m_drawerB.getShowEyePositions());
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnGenerateStrokes(WPARAM wParam, LPARAM lParam)
{
    //OnComputeVertexNormal(wParam, lParam);
    foreach(ei, m_meshA.edges(), NPRMesh::Edges) {
        if (!ei->visited && ei->count > m_drawerB.getMinTol()) {
            Stroke* s = traceStroke(&*ei);
        }
    }

    m_drawerB.setStrokes(&m_strokes);
    CONS("%d strokes generated.\r\n", m_strokes.size());
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnProjectOntoVoxelHull(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    computeSolidVoxel(mesh);
    //OnComputeSolidVoxel(0, 0);
    real mean_error = computeFastMeshMapping(m_strokes, mesh);
    LaplacianSolver::setMeanError(mean_error);
    CONS("Projection done. Ave Dist = %f\r\n", mean_error);
    /*
    if (m_robustMode) {
        time_t t = time(NULL);
        computeAllPairGeodesicDist(mesh);
        t = time(NULL) - t;
        CONS("All pair geodesic distance computed in %d seconds.\r\n", t);
    }
    //smooth(&m_vHull, 1);
    OnComputeVertexNormal(wParam, lParam);

	Array<NPRVertex*> actList;

	actList.reserve(mesh.size());

    int count = 0;
	foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
		NPRVertex* v = &*vi;
        v->m_index = count;
        count++;
		if (!v->connected()) continue;		
		actList.push_back(v);
	}

    foreach (si, m_strokes, Strokes) {
        si->calcStates(actList);
        //si->markCandidates();
        viterbi(mesh, &*si);
        si->processed = true;
    }

    // resolve one vertex maps to multiple vertices case.
    foreach (si, m_strokes, Strokes) {
        foreach (svi, (*si), Stroke) {
            SVertex* sv = &*svi;
            if (sv->vertex->mapped == NULL) {
                sv->vertex->mapped = sv->mapped_vertex;
            } else {
                sv->mapped_vertex = sv->vertex->mapped;
            }
            //sv->mapped_vertex->marked = true;
            //sv->mapped_vertex->addProp(VP_ANCHOR);
        }
    }

    //foreach (vi, m_vHull.vertices(), NPRMesh::Vertices) {
    //    vi->reset();
    //}
    //int num_anchors = interpolateStrokeOnVoxelShell();

    CONS("Projection done.\r\n");
    */
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnDeformVoxelHull(WPARAM wParam, LPARAM lParam)
{
	CriticalSection cs(m_mutex);
    NPRMesh& mesh = getActiveMesh();

    // compute laplacian coordinates, a Point3D per vertex
    computeLaplacianCoord(mesh);
    // smooth the voxel shell
    // solve for deformation
    deform(mesh);

    //vHullToMesh();

    //translateCenter(m_vHull, 0);
    //resizeToUnitBox(m_vHull);
	generateNormals(mesh, 0, 1);// use coords in 0 and store normals in 1

    CONS("Deformation done.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnLoadPoints(WPARAM wParam, LPARAM lParam)
{
    CriticalSection cs(m_mutex);
    Mesh3D& mesh = m_points;

    FILE* fid;
    if (fopen_s(&fid, m_path, "r")) {
        CONS("ERROR: unable to open file %s.\r\n", m_path);
        return;
    }
    if (!loadPoints(fid, mesh)) {
        CONS("ERROR: failed to load point clouds from %s.\r\n", m_path);
        fclose(fid);
        return;
    }
    fclose(fid);

    /*
    CONS("mesh center: %f %f %f\r\n", center_mesh.x(), center_mesh.y(), center_mesh.z());
    CONS("point center: %f %f %f\r\n", center_pt.x(), center_pt.y(), center_pt.z());
    //resizeToUnitBox(mesh);
    m_meshA.getBoundingBox(min_mesh, max_mesh);
    mesh.getBoundingBox(min_pt, max_pt);
    center_pt = (min_pt + max_pt) * 0.5;
    CONS("min: %f %f %f max %f %f %f\r\n",
        min_mesh.x(), min_mesh.y(), min_mesh.z(),
        max_mesh.x(), max_mesh.y(), max_mesh.z());
    CONS("min: %f %f %f max %f %f %f\r\n",
        min_pt.x(), min_pt.y(), min_pt.z(),
        max_pt.x(), max_pt.y(), max_pt.z());
    CONS("point center: %f %f %f\r\n", center_pt.x(), center_pt.y(), center_pt.z());
    */

    assignVertexIndex(mesh);
    joinPieces(mesh);
    resampleStrokes(mesh);
    //real dist = computeAveMinDist(mesh);
    //CONS("Average Minimum Distance: %f\r\n", dist);

    //computeEdges(mesh, 2.1*dist);
    //m_drawerB.setNeighborRadius(dist*2.1);
    //CONS("num edges: %d\r\n", mesh.nEdges());

	mesh.clearSensor().modify();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnSavePoints(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    savePointsV6(mesh);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnVoxelHullDeformToMesh(WPARAM wParam, LPARAM lParam)
{
	CriticalSection cs(m_mutex);

    //computeLaplacianCoord(m_vHull);

    deformNoAnchor(m_vHull);

	generateNormals(m_vHull, 0, 1);// use coords in 0 and store normals in 1

    CONS("Deformation done.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnMapToOriMesh(WPARAM wParam, LPARAM lParam)
{
    static int itr = 0;
    //real ave_dist = computeMeshMapping(m_vHull, m_meshA);
    time_t t = time(NULL);
    computeSolidVoxel(m_meshA);
    real ave_dist = computeFastMeshMapping(m_vHull, m_meshA);
    t = time(NULL) - t;
    CONS("Mapped in %d seconds.\r\n", t);

    LaplacianSolver::setMeanError(ave_dist);
    //if (itr < 3) // remove possible wrong mapping in early iterations
    if (m_ave_dist > 0.03) // when envelope does not converge.
        filterMisMappings(m_vHull);
    scaleMappings(m_vHull);
    itr++;
    CONS("Mapping done. Ave Dist = %f\r\n", ave_dist);
    m_ave_dist = ave_dist;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnSnapToFeatureLines(WPARAM wParam, LPARAM lParam)
{
	CriticalSection cs(m_mutex);
 
    //m_mutex.lock();
    NPRMesh& mesh = getActiveMesh();
    m_robustMode = true;

    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        vi->reset();
    }
    int c = 0;
    foreach (si, m_strokes, Strokes) {
        if (!si->valid) { c++; continue; }
        CONS("%d: ", c);
        //m_mutex.lock();
        cutMesh2(mesh, *si);
        //m_mutex.unlock();
        //MB("Think carefully, act boldly!");
        if (!isValidMesh(mesh)) {
            CONS("mesh invalid...\r\n");
            MB("pause...");
        }
        c++;
    }
    //m_mutex.unlock();
    CONS("Mapping done.\r\n");
    //MB("1");
    //fixMesh(mesh, true);
    //MB("2");
    //removeIsolatedVertices(mesh);
    //MB("3");
    assignVertexIndex(mesh);
    computeSolidVoxel(m_meshA);
    computeFastMeshMapping(mesh, m_meshA);
    if (!isValidMesh(mesh)) {
        CONS("mesh invalid...\r\n");
    }
    computeLaplacianCoord(mesh);
    deformAnchor(mesh);
    CONS("Deformation done.\r\n");
    //MB("4");
    fixMesh(mesh, true);
    cutThinTriangles(mesh);
    fixMesh(mesh, true);
    restoreAnchors(mesh);
    //MB("5");
    removeIsolatedVertices(mesh);
    //MB("6");
    removeIsolatedAnchors(mesh);
    //MB("7");
    CONS("Mesh fixed.\r\n");
    trim(mesh);
    restoreAnchors(mesh);
    CONS("First trimming done.\r\n");
    //MB("7.5");
    connectNearAnchors();
    CONS("Connection done.\r\n");
    //MB("7.6");
    trim(mesh);
    restoreAnchors(mesh);
    CONS("Second trimming done.\r\n");
    //MB("8");
    //m_mutex.unlock();
    resampleFeatureLines(mesh, getAveFeatureEdgeDist(mesh));
    //MB("9");
    removeIsolatedVertices(mesh);
    removeIsolatedAnchors(mesh);
    restoreAnchors(mesh);
    CONS("Resampling lines done.\r\n");
    generateNormals(mesh, 0, 1);// use coords in 0 and store normals in 1
    CONS("Cutting done.\r\n");
    return;

    //--------------------------------------------------
    if (m_robustMode) {
        time_t t = time(NULL);
        computeAllPairGeodesicDist(mesh);
        t = time(NULL) - t;
        CONS("All pair geodesic distance computed in %d seconds.\r\n", t);
    }
    //smooth(&m_vHull, 1);
    OnComputeVertexNormal(wParam, lParam);

	Array<NPRVertex*> actList;

	actList.reserve(mesh.size());

    int count = 0;
	foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
		NPRVertex* v = &*vi;
        v->m_index = count;
        count++;
		if (!v->connected()) continue;		
		actList.push_back(v);
	}

    foreach (si, m_strokes, Strokes) {
        si->calcStates(actList);
        //si->markCandidates();
        viterbi(mesh, &*si);
        si->processed = true;
    }

    // resolve one vertex maps to multiple vertices case.
    foreach (si, m_strokes, Strokes) {
        foreach (svi, (*si), Stroke) {
            SVertex* sv = &*svi;
            if (sv->vertex->mapped == NULL) {
                sv->vertex->mapped = sv->mapped_vertex;
            } else {
                sv->mapped_vertex = sv->vertex->mapped;
            }
            //sv->mapped_vertex->marked = true;
            sv->mapped_vertex->addProp(VP_ANCHOR);
        }
    }
    smooth(&mesh, 5);
    //--------------------------------------------------

    // assign stroke and sv index
    count = 0;
    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        count++;
        int sv_count = 0;
        foreach (svi, (*stroke), Stroke) {
            SVertex* sv = &*svi;
            sv->mapped_vertex->sv = sv;
            sv->mapped_vertex->stroke_index = count;
            sv->mapped_vertex->sv_index = sv_count;
            sv_count++;
        }
    }
    /*
    computeFastMeshMapping(m_strokes, mesh);
    int count = 0;
    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        count++;
        int sv_count = 0;
        foreach (svi, (*stroke), Stroke) {
            SVertex* sv = &*svi;
            ASSERT(sv->mapped_face != NULL);
            real d0 = (sv->mapped_point - sv->mapped_face->v0()->p()).abs();
            real d1 = (sv->mapped_point - sv->mapped_face->v1()->p()).abs();
            real d2 = (sv->mapped_point - sv->mapped_face->v2()->p()).abs();
            NPRVertex* v = NULL;
            if (d0 <= d1 && d0 <= d2) {
                v = sv->mapped_face->v0();
            } else if (d1 <= d0 && d1 <= d2) {
                v = sv->mapped_face->v1();
            } else if (d2 <= d0 && d2 <= d1) {
                v = sv->mapped_face->v2();
            }
            sv->mapped_vertex = v;
            sv->mapped_vertex->sv = sv;
            sv->mapped_vertex->stroke_index = count;
            sv->mapped_vertex->sv_index = sv_count;
            //sv->mapped_point = v->p();
            sv->mapped_vertex->addProp(VP_ANCHOR);
            sv_count++;

            //if (v->secondPos.abs() < epsilon() ||
            //    (v->secondPos - v->p()).abs() >
            //    (sv->vertex->p() - v->p()).abs()) {
            //        v->secondPos = sv->vertex->p();
            //        v->addProp(VP_ANCHOR);
            //}
        }
    }
    */

    /*
    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        SVertex* front = &(stroke->front());
        SVertex* front_second = &(*(++stroke->begin()));
        Point3D dir_front = (front->vertex->p() - front_second->vertex->p()).unit();
        SVertex* back = &(stroke->back());
        SVertex* back_second = &(*(--(--stroke->end())));
        Point3D dir_back = (back->vertex->p() - back_second->vertex->p()).unit();
        dijkstraReset(&mesh);
        List<NPRVertex*> v_list = vertexDijkstraAnchor(front->mapped_vertex, 1, dir_front);
        if (v_list.size() > 0) {
            CONS("connect\r\n");
            SVertex* to_add = v_list.back()->sv;
            ASSERT(to_add != NULL);
            stroke->push_front(*to_add);
        } else {
            CONS("size: %d\r\n", v_list.size());
        }
        v_list.clear();
        dijkstraReset(&mesh);
        v_list = vertexDijkstraAnchor(back->mapped_vertex, 1, dir_back);
        if (v_list.size() > 0) {
            CONS("connect\r\n");
            SVertex* to_add = v_list.back()->sv;
            ASSERT(to_add != NULL);
            stroke->push_back(*to_add);
        } else {
            CONS("size: %d\r\n", v_list.size());
        }
    }
    */

    /*
    int num_anchors = 0;
    foreach(vi, mesh.vertices(), NPRMesh::Vertices) {
        if (vi->isProp(VP_ANCHOR)) num_anchors++;
    }
    */

    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        vi->reset();
    }

    int num_anchors = interpolateStrokeOnVoxelShell();
    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        SVertex* prev = NULL;
        foreach (svi, (*stroke), Stroke) {
            SVertex* sv = &*svi;
            if (svi == stroke->begin()) {
                prev = sv;
                continue;
            }
            NPREdge* e = prev->mapped_vertex->getEdgeTo(sv->mapped_vertex);
            if (e != 0) {
                e->count = m_drawerB.getMaxTol();
                if (e->twin()) {
                    e->twin()->count = m_drawerB.getMaxTol();
                }
            }
            prev = sv;
        }
    }
    //return;

    computeLaplacianCoord(mesh);
    assignVertexIndex(mesh);
    CONS("num_strokes: %d\r\n", m_strokes.size());
    LaplacianSolver solver(mesh.nVertices()*2, mesh.nVertices());
    solver.initialize(&mesh, false);
    foreach(vi, mesh.vertices(), NPRMesh::Vertices) {
        if (vi->isProp(VP_ANCHOR)) {
            solver.begin_row();
            solver.add(vi->m_index, LaplacianSolver::weight);
            solver.set_rhs(
                vi->secondPos.x()*LaplacianSolver::weight,
                vi->secondPos.y()*LaplacianSolver::weight,
                vi->secondPos.z()*LaplacianSolver::weight);
            solver.end_row();
        } else {
            real soft_weight = epsilon();
            real mean_error = 1.0;//LaplacianSolver::getMeanError();
            soft_weight = 1.0 - (vi->p()-vi->nearestPoint).abs()/(2*mean_error);
            soft_weight = max(soft_weight, 0.0);
            soft_weight *= LaplacianSolver::w_target;

            solver.begin_row();
            solver.add(vi->m_index, soft_weight);
            solver.set_rhs(
                vi->nearestPoint.x()*soft_weight,
                vi->nearestPoint.y()*soft_weight,
                vi->nearestPoint.z()*soft_weight);
            solver.end_row();
        }
    }

    solver.solve();
    smooth(&mesh, 3);
    //connectNearAnchors();

    generateNormals(mesh, 0, 1);// use coords in 0 and store normals in 1
    CONS("Snapping done.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnFlipEdge(WPARAM wParam, LPARAM lParam)
{
    CriticalSection cs(m_mutex);
    //m_mutex.lock();
    NPRMesh& mesh = getActiveMesh();
	mesh.computeVertexNormals(0);

    int count = flipEdges(mesh, true, true);
	//fixMesh(mesh, true, true, true, true );
    //m_mutex.unlock();
    //MB("Stop 1");
    //m_mutex.lock();
    //cutThinTriangles(mesh, true);
    //m_mutex.unlock();
    //MB("Stop 2");
    //m_mutex.lock();
    //fixMesh(mesh, true);
    removeIsolatedVertices(mesh);
    restoreAnchors(mesh);

    generateNormals(mesh, 0, 1);// use coords in 0 and store normals in 1
    mesh.computeFaceAreas(0);
    mesh.computeFaceNormals(0);
    mesh.computeVertexNormals(0);

    CONS("%d edges flipped.\r\n", count);
    //m_mutex.unlock();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnCollapseEdge(WPARAM wParam, LPARAM lParam)
{
    CriticalSection cs(m_mutex);
    //m_mutex.lock();
    NPRMesh& mesh = getActiveMesh();
    real aveLength = mesh.getAveEdgeLength();

    int total_count = 0;
    int count = 1;
    while (count != 0) {
        count = collapseShortEdges(mesh, max(0.2*aveLength, m_tol), true);
        total_count += count;
    }
    //fixMesh(mesh, true, true, true, false );
    //m_mutex.unlock();
    //MB("Stop 1");
    //m_mutex.lock();
    //cutThinTriangles(mesh, true);
    //m_mutex.unlock();
    //MB("Stop 2");
    //m_mutex.lock();
    //fixMesh(mesh, true);
    removeIsolatedVertices(mesh);
    restoreAnchors(mesh);

    generateNormals(mesh, 0, 1);// use coords in 0 and store normals in 1
    mesh.computeFaceAreas(0);
    mesh.computeFaceNormals(0);
    mesh.computeVertexNormals(0);

    CONS("%d edges collapsed.\r\n", total_count);
    //m_mutex.unlock();
}


void CNPRAlg::OnCollapseThinTriangles(WPARAM wParam, LPARAM lParam)
{
    CriticalSection cs(m_mutex);
    //m_mutex.lock();
    NPRMesh& mesh = getActiveMesh();

    //m_mutex.unlock();
    //MB("Stop 1");
    //m_mutex.lock();
    cutThinTriangles(mesh, true);
    real aveLength = mesh.getAveEdgeLength();
    int count = 1;
    while (count != 0)
        count = collapseShortEdges(mesh, max(0.2*aveLength, m_tol), true);

    //fixMesh(mesh, true, true, true, false );
    //m_mutex.unlock();
    //MB("Stop 2");
    //m_mutex.lock();
    //fixMesh(mesh, true);
    removeIsolatedVertices(mesh);
    restoreAnchors(mesh);

    generateNormals(mesh, 0, 1);// use coords in 0 and store normals in 1
    mesh.computeFaceAreas(0);
    mesh.computeFaceNormals(0);
    mesh.computeVertexNormals(0);

    CONS("Collapsed thin triangles.\r\n");
    //m_mutex.unlock();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////
struct getDist {
    real operator() (const Point3D& p1, const Point3D& p2) {
        return (p1-p2).abs();
    }
};
void CNPRAlg::OnExtractNormal(WPARAM wParam, LPARAM lParam)
{
    /* cluster testing
    ClusterMachine<double, Point3D, getDist> cm(1);
    for (int i=0; i<8; i++) {
        cm.insert(double(i), Point3D(-0.016818, -0.012624, -0.999779), 1.0);
    }
    cm.insert(double(8), Point3D(0.011546, -0.034119, -0.999351), 1.0);

    Point3D result;
    real min_error = bigval();
    for (int j=0; j<10; j++) {
        real err = cm.process(10, j!=0);
        //CONS("err = %f\r\n", err);
        if (err >= min_error) continue;
        min_error = err;
        result = cm.getCenter(0);
    }
    CONS("%f %f %f\r\n", result.x(), result.y(), result.z());
    return;
    */
    NPRMesh& mesh = getActiveMesh();
    //computeSolidVoxel(m_meshA);
    //m_meshA.computeFaceNormals(0);
    //m_meshA.computeFaceAreas(0);
    mesh.computeFaceNormals(0);
    mesh.computeFaceAreas(0);
    mesh.computeVertexNormals(0);
#if 0
    //computeFastMeshMapping(mesh, m_meshA);
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = &*vi;
        if (v->isProp(VP_ANCHOR)) {
            //ClusterMachine<NPRFace*, Point3D, getDist> cm(2);
#if 0
            List<NPRFace*> f_list = getRings(v, 3);
            foreach (fi, f_list, List<NPRFace*>) {
                /*
                NPREdge* e = (*fi)->e0();
                Point3D normal;
                //real weight;
                do {
                    NPRFace* project_f = (NPRFace*)(e->s()->mapped_face);
                    if (project_f->m_normal * (*fi)->m_normal < 0) {
                        project_f->m_normal *= -1;
                    }
                    normal += project_f->m_normal * project_f->m_area;
                    //weight += project_f->m_area;
                    e = e->nextInFace();
                } while (e != (*fi)->e0());
                normal.normalize();
                //normal /= weight;
                */
                //cm.insert(*fi, normal, (*fi)->m_area);
                real weight = (*fi)->m_area/((*fi)->m_ring*(*fi)->m_ring);
                cm.insert(*fi, (*fi)->m_normal, weight);
                (*fi)->m_ring = 0;
            }
#else
            List<NPRVertex*> v_list = getNearByVertices(mesh, v, 0.10);
            int partitions = partitionFaces(v, 0.15);
            //CONS("%d partitions\r\n", partitions);
            if (partitions < 2) continue;
            Array<ClusterMachine<void*, Point3D, getDist> > cms(partitions, 1);
            std::set<NPRFace*> f_list;
            List<Point3D> normal_debug;
            foreach (vi, v_list, List<NPRVertex*>) {
                NPRVertex* vertex = *vi;
                NPREdge::circulator ci = vertex->edge_circulator();
                do {
                    NPRFace* f = (*ci)->face();
                    if (f->m_ring != 0 && f->m_ring <= partitions) {
                        //cms[f->m_ring-1].insert(vertex, vertex->normal, f->m_area/3.0);
                        //if (v->m_index == 1167)
                        //    CONS("vn %d: %f %f %f\r\n", f->m_ring, vertex->normal.x(), vertex->normal.y(), vertex->normal.z());
                        f_list.insert(f);
                        //normal_debug.push_back(v->normal);
                        //if (isnan(v->normal.abs())) {
                        //    v->marked = true;
                        //}
                    }
                } while (++ci != vertex->edge_circulator() && ci != 0);
            }
            {
                NPREdge::circulator ci = v->edge_circulator();
                do {
                    NPRFace* f = (*ci)->face();
                    if (f->m_ring != 0 && f->m_ring < partitions) {
                        f_list.insert(f);
                    }
                } while (++ci != v->edge_circulator() && ci != 0);
            }
            foreach (fi, f_list, std::set<NPRFace*>) {
                NPRFace* f = *fi;
                real w = f->m_area;
                if (w < epsilon()) continue;
                cms[f->m_ring-1].insert(f, f->m_normal, w);
                //normal_debug.push_back(f->m_normal);
                //if (isnan(f->m_normal.abs())) {
                //    f->color() = Color3d(1.0, 0.0, 0.0);
                //}
                //if (v->m_index == 1166) {
                //    CONS("fn: %f %f %f\r\n", f->m_normal.x(), f->m_normal.y(), f->m_normal.z());
                //}
            }
            Array<Point3D> normals(partitions);
            bool problem = false;
            for (int i=0; i<partitions; i++) {
                real min_error = bigval();
                for (int j=0; j<10; j++) {
                    real err = cms[i].process(10, j!=0);
                    //if (v->m_index == 1167) {
                    //    Point3D n = cms[i].getCenter(0);
                    //    CONS("%d err = %f  n(%f %f %f)\r\n",
                    //        i, err, n.x(), n.y(), n.z());
                    //}
                    if (err >= min_error) continue;
                    min_error = err;
                    normals[i] = cms[i].getCenter(0);
                }
                if (isnan(min_error)) problem = true;
                //CONS("%d: min_error = %f\r\n", i, min_error);
            }
            /*
            for (int i=0; i<partitions; i++) {
                //CONS("data size: %d\r\n", cms[i].size());
                //CONS("n: %f %f %f\r\n",
                //    normals[i].x(), normals[i].y(), normals[i].z());
                if (normals[i].abs() < epsilon()) {
                    problem = true;
                }
            }
            */
            /*
            if (problem) {
                int i=0;
                foreach (ni, normal_debug, List<Point3D>) {
                    CONS("  %d: %f %f %f\r\n", i, ni->x(), ni->y(), ni->z());
                    i++;
                }
            }
            */
            v->marked = true;
            //MB("...");
            v->marked = false;
            NPREdge::circulator ci = v->edge_circulator();
            NPREdge::circulator begin = ci;
            do {
                NPREdge* e = *ci;
                if (e->count > 0 || (e->twin() && e->twin()->count > 0)) {
                    begin = ci;
                    break;
                }
            } while(++ci != v->edge_circulator() && ci != 0);
            ci = begin;
            int partition_num = 0;
            do {
                NPREdge* e = *ci;
                partition_num = e->face()->m_ring;
                ASSERT(partition_num > 0 && partition_num <= partitions);
                e->normal = normals[partition_num-1];
            } while (++ci != begin && ci != 0);

#endif
            /*
            real min_error = bigval();
            for (int i=0; i<10; i++) {
                real err = cm.process(10, i!=0);
                if (err >= min_error) continue;
                //std::set<Point3D> unique_normals;
                foreach (fi, f_list, List<NPRFace*>) {
                    Point3D n = cm.getCenter(*fi);
                    n.normalize();
                    //unique_normals.insert(n);
                    //CONS("normal: %f %f %f.\r\n", n.x(), n.y(), n.z());
                    v->m_normals[*fi] = n;
                }
                //CONS("Unique normals: %d\r\n", unique_normals.size());
            }
            */
        }
    }
#else
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        if (!ei->twin()) continue;
        if (ei->count > 0 || ei->twin()->count > 0) {
            ei->normal = ei->face()->m_normal;
        }
    }
#endif
    for (int i=0; i<10; i++) {
        smoothFeatureLineNormal(mesh);
    }
    assignAnchorNormal(mesh);
    registerNormals(mesh);
    CONS("Normal extraction done.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnComputeLaplacianVector(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    computeLaplacianCoord(mesh);
    CONS("Done computing Lapacian vectors.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnRefineMesh(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    //CriticalSection cs(m_mutex);
    refineMesh2(mesh);
    mesh.computeFaceAreas(0);
    mesh.computeFaceNormals(0);
    generateNormals(mesh, 0, 1);// use coords in 0 and store normals in 1
    CONS("Done refine mesh.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnFilterUseNormals(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    filterAnchors(mesh, m_angleTol);
    trim(mesh);
    removeIsolatedAnchors(mesh);
    CONS("Done filtering using normals.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnPartition(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    mesh.computeFaceNormals(0);
    mesh.computeFaceAreas(0);
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        if (edge->count > 0) edge->count = 180;
    }

    for (int i=0; i<5; i++) {
        computeDistToFeature(mesh);
        foreach (ei, mesh.edges(), NPRMesh::Edges) {
            NPREdge* edge = &*ei;
            if (edge->count < 10) edge->count = 0;
        }
        List<NPRFace*> f_list = extractLocalMax(mesh);
        foreach (fi, f_list, List<NPRFace*>) {
            NPRFace* face = *fi;
            face->color() = Color3d(1.0, 0.0, 0.0);
        }
        MB("Pause");
        partition(mesh, f_list);
        MB("Iteration %d done", i);
    }
    CONS("Done partitioning the mesh.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnRemesh(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    mesh.computeFaceNormals(0);
    mesh.computeFaceAreas(0);
    mesh.computeVertexNormals(0);
    computeLaplacianCoord(mesh);

    CriticalSection cs(m_mutex);

    foreach (fi, mesh.faces(), NPRMesh::Faces) {
        fi->color() = Color3d(1.0, 1.0, 1.0);
    }

    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = &*vi;
        Point3D d = v->lCoord - v->normal * (v->normal * v->lCoord);
        v->secondPos = v->p() + d;
    }


    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = &*vi;
        //v->marked = false;
        NPREdge::circulator ci = v->edge_circulator();
        bool good = true;
        Color3d c;
        do {
            NPRFace* f = (*ci)->face();
            if (!smoothCheck(f->v0()->secondPos, f->v1()->secondPos, f->v2()->secondPos,
							 f->v0()->normal, f->v1()->normal, f->v2()->normal, m_smooth_tol)) {
                good = false;
                c = Color3d(1.0, 0.8, 0.8);
                break;
            }
            if (!distCheck(f, m_dist_tol)) {
                good = false;
                c = Color3d(1.0, 1.0, 0.8);
                break;
            }
            f->color() = Color3d(1.0, 0.0, 0.0);
        } while (++ci != v->edge_circulator() && ci != 0);

        if (good) {
            v->p() = v->secondPos;
            ci = v->edge_circulator();
            do {
                NPRFace* f = (*ci)->face();
                f->color() = Color3d(1.0, 0.0, 0.0);
            } while (++ci != v->edge_circulator() && ci != 0);
        } else {
            ci = v->edge_circulator();
            do {
                NPRFace* f = (*ci)->face();
                f->color() = c;
            } while (++ci != v->edge_circulator() && ci != 0);
        }
    }

    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = &*vi;
        v->secondPos *= 0;
    }

    generateNormals(mesh, 0, 1);// use coords in 0 and store normals in 1
    CONS("Done remeshing.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnCutLongEdge(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    real dist = m_ave_edge_len;
    if (dist < epsilon()) {
        dist = mesh.getAveEdgeLength();
    }
    CriticalSection cs(m_mutex);
    cutLongEdges(mesh, dist*1.5);
    generateNormals(mesh, 0, 1);// use coords in 0 and store normals in 1
    mesh.computeFaceAreas(0);
    mesh.computeFaceNormals(0);
    mesh.computeVertexNormals(0);

    CONS("Done cutting mesh.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnRecoverRidge(WPARAM wParam, LPARAM lParam)
{	
	NPRMesh& vHull = m_vHull,  & refMesh = m_meshA;
	EdgeMap M;
	
	// used before calling mapping functions
	OnMapToOriMesh(wParam, lParam);
	vHull.computeFaceNormals(0);
    vHull.computeFaceAreas(0);
    vHull.computeVertexNormals(0);


	// reset color
	foreach( fi, vHull.faces(), NPRMesh::Faces )
		fi->color() = Color3d(0.9, 0.9, 0.9);
	foreach( ei, vHull.edges(), NPRMesh::Edges)
		ei->count = 0;


	// find flippable edges
	foreach( ei, vHull.edges(), NPRMesh::Edges)
	{
		double diff;
		NPREdge* e = &*ei;
		if( isEdgeFlippable(e, refMesh, diff ) )
		{
			M.insert( std::make_pair(e, abs(diff)) );
			e->count = 90;
		}	
		else
		{
			e->count = 0;
		}
	}

	MB("proceed");
	CriticalSection cs(m_mutex);
	

	while( !M.empty() )
	{
		NPREdge* e; 
		double diffDist;
		
		// find minimum
		real maxValue = DBL_MIN;
		EdgeMap::iterator found, it = M.begin();
		while( it != M.end() )
		{
			if(  maxValue < it->second )
			{
				maxValue = it->second;
				e = it->first;
				found = it;
			}
			it++;
		}
		M.erase(found); 
		if( M.find(e->twin()) != M.end() )
			M.erase(M.find(e->twin()));


		//e->count = 50;
		//CONS("Size of map %d \r\n", M.size() );
		//if( M.size() == 761 )
		//	MB("stop");


		// flip the edge
		if( isEdgeFlippable(e, refMesh, diffDist ) )
		{
			NPREdge* newe = vHull.flipEdge(e);
			newe->count = 20;

			NPREdge* tempe = newe->nextInFace();
			while( tempe != newe )
			{
				if( isEdgeFlippable(tempe, refMesh, diffDist) )
					M[tempe] = abs(diffDist);
				else if( M.find(tempe) != M.end() )
				{
					M.erase( M.find(tempe) );
					if( M.find(tempe->twin()) != M.end() )
						M.erase( M.find(tempe->twin()) );
				}
				tempe = tempe->nextInFace();
			}


			tempe = newe->twin()->nextInFace();
			while( tempe !=  newe->twin() )
			{
				if( isEdgeFlippable(tempe, refMesh, diffDist) )
					M[tempe] = abs(diffDist);
				else if( M.find(tempe) != M.end() )
				{
					M.erase( M.find(tempe) );
					if( M.find(tempe->twin()) != M.end() )
						M.erase( M.find(tempe->twin()) );
				}
				tempe = tempe->nextInFace();
			}
		}
	}

	CONS("Flipping done \r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnClear(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    foreach(ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        edge->count = 0;
    }

    foreach(vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = &*vi;
        v->reset();
    }
    CONS("Clear done.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnDetect3Loop(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    real len = mesh.getAveEdgeLength();

    /*// uncomment to view the 3-loops
    foreach(ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        if (edge->length() > len*0.2) continue;
        NPRVertex* s = edge->s();
        NPRVertex* t = edge->t();
        NPREdge::circulator si = s->edge_circulator();
        do {
            NPREdge* s_edge = *si;
            NPREdge::circulator ti = t->edge_circulator();
            do {
                NPREdge* t_edge = *ti;
                if (s_edge->t() == t_edge->t() &&
                    mesh.getFace(s, t, s_edge->t()) == 0) {
                        // Detected a 3-loop
                        edge->count++;
                        s_edge->count++;
                        t_edge->count++;
                }
            } while (++ti != t->edge_circulator() && ti != 0);
        } while (++si != s->edge_circulator() && si != 0);
    }

    MB("test");
    */

    break3Loops(mesh, 0.5*len);
    mesh.removeSmallComponents(mesh.nFaces()/10);
    CONS("done.\r\n");
    return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnSmoothSegmentNormal(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    real ave_len = mesh.getAveEdgeLength();
    for (int i=0; i<10; i++)
        smoothFeatureSegmentNormal(mesh, 2*ave_len);
    assignAnchorNormal(mesh);
    registerNormals(mesh);
    CONS("Smooth segment normal done.\r\n");
}



/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnAddNoiseToModelVertices(WPARAM wParam, LPARAM lParam)
{
	CONS("Adding noise %f percent of the bounding box diagnol \r\n", getNoiseValue() );

	srand( (unsigned)time( NULL ) );

    NPRMesh& mesh = getActiveMesh();

	Point3D min_mesh, max_mesh;
	mesh.getBoundingBox(min_mesh, max_mesh);

	double diagnol_length =  ( max_mesh - min_mesh ).abs();
	double noise_length = ( getNoiseValue() * diagnol_length )/100.f;
	CONS("Diagnol length %f,   Noise length %f \r\n", diagnol_length, noise_length );
	
	// add noise per vertex 
	foreach (vi, mesh.vertices(), NPRMesh::Vertices) 
	{
		double noise =  ( (double)rand()/ RAND_MAX ) * noise_length;

		Point3D direction;
		direction.x() = (( (double)rand()/RAND_MAX ) - 0.5 )* 2 ; // -1 to 1
		direction.y() = (( (double)rand()/RAND_MAX ) - 0.5 )* 2 ; // -1 to 1
		direction.z() = (( (double)rand()/RAND_MAX ) - 0.5 )* 2 ; // -1 to 1

		direction.normalize();
		direction = direction * noise;

		//CONS("noise added %f %f %f \r\n", direction.x(), direction.y(), direction.z() );
		vi->p() = vi->p() + direction;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnAddNoiseToModelFaces(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();

	Point3D min_mesh, max_mesh;
	mesh.getBoundingBox(min_mesh, max_mesh);

	double diagnol_length =  ( max_mesh - min_mesh ).abs();
	double noise_length = ( getNoiseValue() * diagnol_length )/100.f;

}


/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnCorrectSelfIntersection(WPARAM wParam, LPARAM lParam)
{
    NPRMesh& mesh = getActiveMesh();
    mesh.computeFaceNormals(0);
    mesh.computeFaceAreas(0);
    mesh.computeVertexNormals(0);
    computeSolidVoxel(mesh);
    for (int i=0; i<m_vgrid.getWidth(); i++) {
        for (int j=0; j<m_vgrid.getHeight(); j++) {
            for (int k=0; k<m_vgrid.getDepth(); k++) {
                std::set<NPRFace*> faces;
                for (int ii=i; ii<=i+1; ii++) {
                    if (ii >= m_vgrid.getWidth()) continue;
                    for (int jj=j; jj<=j+1; jj++) {
                        if (jj >= m_vgrid.getHeight()) continue;
                        for (int kk=k; kk<=k+1; kk++) {
                            if (kk >= m_vgrid.getDepth()) continue;
                            Voxel* vox = m_vgrid.vox(ii,jj,kk);
                            faces.insert(vox->f_list.begin(), vox->f_list.end());
                        }
                    }
                }
                bool noSelfIntersection = true;
                int count = 0;
                do {
                    noSelfIntersection = true;
                    std::set<NPRFace*>::iterator itr = faces.begin();
                    while (itr != faces.end()) {
                        NPRFace* f1 = *itr;
                        std::set<NPRFace*>::iterator itr2(itr);
                        while (itr2 != faces.end()) {
                            NPRFace* f2 = *itr2;
                            Point3D n1 = f1->m_normal;
                            Point3D n2 = f2->m_normal;
                            if (f2->adjacentTo(f1) || f1 == f2 || n1*n2 > epsilon()) {
                                itr2++;
                                continue;
                            }
                            float v0[3], v1[3], v2[3];
                            float u0[3], u1[3], u2[3];
                            for (int l=0; l<3; l++) {
                                v0[l] = f1->v0()->p()[l]*100;
                                v1[l] = f1->v1()->p()[l]*100;
                                v2[l] = f1->v2()->p()[l]*100;
                                u0[l] = f2->v0()->p()[l]*100;
                                u1[l] = f2->v1()->p()[l]*100;
                                u2[l] = f2->v2()->p()[l]*100;
                                //CONS("%f %f %f <-> %f %f %f\r\n", v0[l], v1[l], v2[l], u0[l], u1[l], u2[l]);
                            }
                            if (TRI_UTIL::tri_tri_intersect(v0, v1, v2, u0, u1, u2) != 0) {
                                correctSelfIntersection(f1, f2);
                                noSelfIntersection = false;
                            }
                            itr2++;
                        }
                        itr++;
                    }
                    count++;
                    // Re-check the voxel until no self intersections left.
                } while (!noSelfIntersection && count < 10);
            }
        }
    }
    CONS("Correct self-intersection done.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnSaveColor(WPARAM wParam, LPARAM lParam)
{
	CriticalSection cs(m_mutex);
    Mesh3D& mesh = getActiveMesh();
    FILE *fid = 0;
    if (fopen_s(&fid, m_path, "w")) {
        TRACE("ERROR: unable to open file %s\r\n", m_path);
    }

    saveColorMap(fid, mesh);

    fclose(fid);
}


void CNPRAlg::OnMDS(WPARAM wParam, LPARAM lParam)
{
	CONS("Starting MDS from %d to %d .... \r\n", getStartIndex(), getEndIndex());

	mxArray *inputFile, *outputFile, *pointsFile;
	mxArray *start, *end;
	
	/* Call the MCR and library initialization functions */
	if( !mclInitializeApplication(NULL,0) )
	{
		CONS("Could not initialize the application. \r\n");
		exit(1);
	}

	if (!MDSlibInitialize())
	{
		CONS("Could not initialize the library. \r\n");
		exit(1);
	}

	/* Create an mxArray to input into mlfFoo */
	inputFile = mxCreateString(RESOLVE_DIR_NAME);
	outputFile = mxCreateString(CONSTRAINED_3D_BOUNDARY_TRI_DIR_NAME);
	pointsFile = mxCreateString(CURVE_DIR);
	start = mxCreateScalarDouble( getStartIndex() );
	end = mxCreateScalarDouble( getEndIndex() );

	/* Call the implementation function */
	/* Note the second input argument should be &y_ptr instead of y_ptr. */
	mlfRunDelaunay2( inputFile, outputFile, pointsFile, start, end );

	/* Call the library termination function */
	MDSlibTerminate();

	mxDestroyArray(inputFile);
	mxDestroyArray(outputFile);
	mxDestroyArray(pointsFile);
	mxDestroyArray(start);
	mxDestroyArray(end);

	mclTerminateApplication();

	CONS("MDS Complete \r\n");
}

void CNPRAlg::OnLoadColor(WPARAM wParam, LPARAM lParam)
{
	CriticalSection cs(m_mutex);
    Mesh3D& mesh = getActiveMesh();
    computeSolidVoxel(mesh);
    FILE *fid = 0;
    if (fopen_s(&fid, m_path, "r")) {
        TRACE("ERROR: unable to open file %s\r\n", m_path);
    }

    loadColorMap(fid, mesh);

    fclose(fid);
}


void CNPRAlg::OnLoadAbstractionNet(WPARAM wParam, LPARAM lParam)
{
	CriticalSection cs(m_mutex);
    
	FILE *fid = 0;
    if (fopen_s(&fid, m_path, "r")) {
        TRACE("ERROR: unable to open file %s\r\n", m_path);
		return;
    }
    fclose(fid);

	std::vector<CURVE> curves;
	const char* fileName = m_path;

	int opened = openCurveFile(curves, fileName);

	if( opened ) 
	{
		resolveAllCurves(curves);

		setStartIndex(0);
		setEndIndex(curves.size());

		curves.clear();

		CONS("Curves Resolved \r\n");
	}
}



void CNPRAlg::OnSpin(WPARAM wParam, LPARAM lParam) 
{
	m_drawerA.setSecondView( true );

	Point3D y(0,1,0); // up direction
	m_drawerA.animate_spin_r = y;

	//m_drawerA.setEyePosition(Point3D(0, 1, 1));

	const int steps = 200;
	m_drawerA.animate_spin_asum = 0.0;
	
	//for (int i = 1; i <= steps; ++i) {
	while (m_drawerA.animate_spin_asum <= 360.0) {
		m_drawerA.animate_spin_a = 360.0/steps;
		m_meshA.appearanceSensor().modify();
		Sleep(30);
	}
	//m_stretchUI.animate_spin_a = -1;

	//m_drawerA.setSecondView( false );

}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnGenerateEnvelope(WPARAM wParam, LPARAM lParam)
{
    int count = 0;
    do {
        OnComputeLaplacianVector(wParam, lParam);
        OnMapToOriMesh(wParam, lParam);
        OnVoxelHullDeformToMesh(wParam, lParam);
        //MB("deform done");
        OnRemesh(wParam, lParam);
        //MB("Remesh done");
        OnCollapseEdge(wParam, lParam);
        //MB("Collapse done");
        OnFlipEdge(wParam, lParam);
        //MB("Flip done");
        OnCutLongEdge(wParam, lParam);
        //MB("Cut long edge done");
        OnCollapseThinTriangles(wParam, lParam);
        CONS("Generating Envelope itr %d: %f error.\r\n", count+1, m_ave_dist);

        char filename[30];
        if (sprintf_s(filename, "C:/envelope_itr_%02d.wrl", count+1) < 0) {
            CONS("Error: unable to generate envelope name.\r\n");
        } else if (!saveVRML(filename, m_vHull)) {
            CONS("Failed to save mesh.\r\n");
        } else {
            CONS("Mesh saved to %s.\r\n", filename);
        }

        count++;
        if (count >= 20) break; // Too many iterations
        //MB("o");
    } while (m_ave_dist > m_ave_dist_tol);
    CONS("Done generating envelope in %d iterations.\r\n", count);
    CONS("Target error: %f   Actual error: %f\r\n",
        m_ave_dist_tol, m_ave_dist);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * markNPREdges: set the nprEdge flag for crease/boundary/silhouette.
 */
void CNPRAlg::markNPREdges() {
    Mesh3D* m_mesh = &(getActiveMesh());
    Point3D z = m_drawerA.getViewDir();
    int pid = 0;
    foreach (ei, m_mesh->edges(), NPRMesh::Edges) {
        if (ei->eCrease || ei->twin() == 0) {
            ei->addProp(EP_NPREDGE);
            //ei->nprEdge = true;
        } else {
            Point3D n0 = triangleFaceNormal(ei->face(), pid);
            Point3D n1 = triangleFaceNormal(ei->twin()->face(), pid);

            if (n0*z > -epsilon() && n1*z < epsilon()) {
                ei->addProp(EP_NPREDGE);
                //ei->nprEdge = true;
            } else {
                ei->remProp(EP_NPREDGE);
                //ei->nprEdge = false;
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * IsVisible: check if the target point is visible by ray-tracing.
 */
bool CNPRAlg::IsVisible(const Point3D& viewDir, const Point3D& target) const{
    foreach_const (fi, m_meshA.faces(), NPRMesh::Faces) {
        Point3D p = fi->v0()->p();
        Point3D n = triangleFaceNormal(&*fi);
        double t = (p-target)*n/(viewDir*n);
        if (t > -epsilon()) {
            continue;
        } else {
            // check intersection is inside the triangle
            Point3D intersection = target+viewDir*(t);
            Point3D c1 = cross(intersection - fi->v0()->p(),
                fi->v1()->p() - fi->v0()->p());
            Point3D c2 = cross(intersection - fi->v1()->p(),
                fi->v2()->p() - fi->v1()->p());
            Point3D c3 = cross(intersection - fi->v2()->p(),
                fi->v0()->p() - fi->v2()->p());
            if (c1*c2 >= 0 && c2*c3 >= 0 && c1*c3 >= 0 ||
                c1*c2 <= 0 && c2*c3 <= 0 && c1*c3 <= 0) {
                    return false;
            }
        }
    }
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::createUniformEyePositions(int level) {
	// if you want to create a new list of eye positions, be sure that 
	// clear the original list of eye positions first;
	if (m_eye_positions.empty() == false) return;

	Array<Point2D> sphi_quads;
	sphi_quads.push_back(Point2D(0, 0));
	sphi_quads.push_back(Point2D(1, 0));
	sphi_quads.push_back(Point2D(1, 1));
	sphi_quads.push_back(Point2D(0, 1));

	// subdivision 
	for (int i = 0; i < level; ++i)
	{
		Array<Point2D> curr_quads(sphi_quads);
		//curr_quads.assign(sphi_quads.begin(), sphi_quads.end());
		
		sphi_quads.clear();
		for (int j = 0; j < curr_quads.size(); j += 4)
		{
			Point2D p[4], p_e[4];
			for (int k = 0; k < 4; ++k) p[k]	= curr_quads[j + k];
			for (int k = 0; k < 4; ++k) 
			{
				p_e[k].x() = (0.5 * p[k].x() + 0.5 * p[(k + 1) % 4].x());
				p_e[k].y() = (0.5 * p[k].y() + 0.5 * p[(k + 1) % 4].y());
			}
			Point2D p_c = Point2D(
				0.25 * (p[0].x() + p[1].x() + p[2].x() + p[3].x()),
				0.25 * (p[0].y() + p[1].y() + p[2].y() + p[3].y()));

			for (int k = 0; k < 4; ++k)
			{
				sphi_quads.push_back(p[k]);
				sphi_quads.push_back(p_e[k]);
				sphi_quads.push_back(p_c);
				sphi_quads.push_back(p_e[(k - 1 + 4) % 4]);
			}
		}
	}

    //FILE* fid = fopen("view.txt", "w");
	for (int i = 0; i < sphi_quads.size(); i += 4)
	{
		Point2D& p1 = sphi_quads[i];
		Point2D& p2 = sphi_quads[i + 1];
		Point2D& p3 = sphi_quads[i + 2];
		Point2D& p4 = sphi_quads[i + 3];

		// centroid 
		double x = (p1.x() + p2.x() + p3.x() + p4.x()) / 4.0;
		double y = (p1.y() + p2.y() + p3.y() + p4.y()) / 4.0;
		double z;

		Point3D eye = m_sphtri.Chart(x, y) * 2.0;
		x = eye.x(); y = eye.y(); z = eye.z();

		m_eye_positions.push_back(Point3D( x,  y,  z));
		m_eye_positions.push_back(Point3D(-x,  y,  z));
		m_eye_positions.push_back(Point3D( x, -y,  z));
		m_eye_positions.push_back(Point3D( x,  y, -z));
		m_eye_positions.push_back(Point3D(-x, -y,  z));
		m_eye_positions.push_back(Point3D(-x,  y, -z));
		m_eye_positions.push_back(Point3D( x, -y, -z));
		m_eye_positions.push_back(Point3D(-x, -y, -z));
        //fprintf(fid, "%f %f %f\r\n", x, y, z);
        //fprintf(fid, "%f %f %f\r\n", -x, y, z);
        //fprintf(fid, "%f %f %f\r\n", x, -y, z);
        //fprintf(fid, "%f %f %f\r\n", x, y, -z);
        //fprintf(fid, "%f %f %f\r\n", -x, -y, z);
        //fprintf(fid, "%f %f %f\r\n", -x, y, -z);
        //fprintf(fid, "%f %f %f\r\n", x, -y, -z);
        //fprintf(fid, "%f %f %f\r\n", -x, -y, -z);
	}
    //fclose(fid);
}
/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::resetEdge(NPRMesh& mesh) {
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        ei->count = 0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::resetFace(NPRMesh& mesh) {
    foreach (fi, mesh.faces(), NPRMesh::Faces) {
        fi->m_visCount = 0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::resetEyePosition() {
    m_drawerA.setEyePosition(NPRDrawer::DEFAULT_EYE_POSITION);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadEdgeList(FILE* fid) {
    /*
    TRACE("Loading %s \r\n", filename);
    FILE *fid = 0;
    fid = fopen(filename, "r");
    if (!fid) {
        TRACE("ERROR: unable to open file %s\r\n", filename);
        return;
    }
    
    int version=1;
    if (fscanf_s(fid, "version %d ", &version) != 1) {
        CONS("ERROR: no version string at the beginning.\r\n");
        fclose(fid);
        return;
    }
    */

    NPRMesh& mesh = getActiveMesh();
    Array<EdgeListElement> edgeList;
    while (!feof(fid)) {
        float p[3];
        if (fscanf_s(fid, "P %f %f %f ", &p[0], &p[1], &p[2]) == EOF)
        { break; }
        Point3D p1(p[0], p[1], p[2]);
        if (fscanf_s(fid, "P %f %f %f ", &p[0], &p[1], &p[2]) == EOF) 
        { break; }
        Point3D p2(p[0], p[1], p[2]);
        //CONS("p1: %f %f %f\r\n", p1.x(), p1.y(), p1.z());
        //CONS("p2: %f %f %f\r\n", p2.x(), p2.y(), p2.z());
        EdgeListElement e(p1,p2);
        long pos = ftell(fid);
        while(fgetc(fid) == 'N') {
            fseek(fid, pos, SEEK_SET);
            if (fscanf_s(fid, "N %f %f %f ", &p[0], &p[1], &p[2]) == EOF)
            { break; }
            Point3D n(p[0], p[1], p[2]);
            //CONS("n %f %f %f\r\n", n.x(), n.y(), n.z());
            e.addNormal(n);
            pos = ftell(fid);
        }
        fseek(fid, pos, SEEK_SET);
        edgeList.push_back(e);
    }
    //fclose(fid);

    //m_drawerB.setEdgeList(edgeList);
    updateEdgeCount(edgeList);
    restoreAnchors(mesh);
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadEdgeListVn1(FILE *fid)
{
    NPRMesh& mesh = getActiveMesh();
    computeSolidVoxel(mesh);

    // Parse points
    int num_pts;
    if (fscanf_s(fid, "%d", &num_pts) == EOF) {
        CONS("Cannot parse number of points.\r\n");
        return false;
    }
    CONS("%d points.\r\n", num_pts);
    Array<NPRVertex*> v_array(num_pts, NULL);
    for (int i=0; i<num_pts; i++) {
        float p[3];
        if (fscanf_s(fid, " P %f %f %f ", &p[0], &p[1], &p[2]) == EOF)
        {
            CONS("Error: Cannot parse points.\r\n");
            return false;
        }
        //CONS("P %f %f %f\r\n", p[0], p[1], p[2]);
        Point3D point(p[0], p[1], p[2]);
        Point3D mapped;
        NPRFace* face = getNearestPoint(point, mesh, mapped);
        if (face == NULL) {
            CONS("Error: cannot find a close match.\r\n");
            return false;
        }
        foreach (ei, face->edges(), NPRFace::Edges) {
            NPREdge* e = *ei;
            if ((e->s()->p() - point).abs() < numeric_limits<float>::epsilon()) {
                v_array[i] = e->s();
                break;
            }
            if ((e->twin()->nextInFace()->t()->p() - point).abs() < numeric_limits<float>::epsilon()) {
                v_array[i] = e->twin()->nextInFace()->t();
                break;
            }
        }
        if (v_array[i] == NULL) {
            CONS("Error: no vertex at position [%f %f %f].\r\n",
                p[0], p[1], p[2]);
            face->color() = Color3d(1.0, 0.0, 0.0);
            face->v0()->marked = true;
            face->v1()->marked = true;
            face->v2()->marked = true;
            return false;
        }
    }

    // Parse feature edges.
    int num_edges;
    if (fscanf_s(fid, "%d", &num_edges) == EOF) {
        CONS("Cannot parse number of edges.\r\n");
        return false;
    }
    CONS("%d edges.\r\n", num_edges);
    for (int i=0; i<num_edges; i++) {
        int s_index, t_index;
        if (fscanf_s(fid, " E %d %d ", &s_index, &t_index) == EOF) {
            CONS("Error: cannot parse edge.\r\n");
            return false;
        }
        NPREdge* e = v_array[s_index]->getEdgeTo(v_array[t_index]);
        if (e == NULL || e->twin() == NULL) {
            CONS("Error: cannot find edge %d %d.\r\n", s_index, t_index);
            return false;
        }
        e->count = m_drawerB.getMaxTol();
        e->twin()->count = m_drawerB.getMaxTol();
        float n1[3]; float n2[3];
        if (fscanf_s(fid, " N %f %f %f ", &n1[0], &n1[1], &n1[2]) == EOF ||
            fscanf_s(fid, " N %f %f %f ", &n2[0], &n2[1], &n2[2]) == EOF) {
                CONS("Error: cannot parse normal of edge %d %d.\r\n",
                    s_index, t_index);
                return false;
        }
        e->normal = Point3D(n1[0], n1[1], n1[2]);
        e->twin()->normal = Point3D(n2[0], n2[1], n2[2]);
    }

    // Parse virtual edges.
    int num_v_edges;
    if (fscanf_s(fid, "%d", &num_v_edges) == EOF) {
        CONS("Cannot parse number of virtual edges.\r\n");
        return false;
    }
    CONS("%d virtual edges.\r\n", num_v_edges);
    for (int i=0; i<num_v_edges; i++) {
        int s_index, t_index;
        if (fscanf_s(fid, " E %d %d ", &s_index, &t_index) == EOF) {
            CONS("Error: cannot parse edge.\r\n");
            return false;
        }
        NPREdge* e = v_array[s_index]->getEdgeTo(v_array[t_index]);
        if (e == NULL || e->twin() == NULL) {
            CONS("Error: cannot find edge %d %d.\r\n", s_index, t_index);
            return false;
        }
        e->count = 1;
        e->twin()->count = 1;
        float n1[3]; float n2[3];
        if (fscanf_s(fid, " N %f %f %f ", &n1[0], &n1[1], &n1[2]) == EOF ||
            fscanf_s(fid, " N %f %f %f ", &n2[0], &n2[1], &n2[2]) == EOF) {
                CONS("Error: cannot parse normal of edge %d %d.\r\n",
                    s_index, t_index);
                return false;
        }
        e->normal = Point3D(n1[0], n1[1], n1[2]);
        e->twin()->normal = Point3D(n2[0], n2[1], n2[2]);
    }
    restoreAnchors(mesh);
    CONS("Parsing done.\r\n");
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadEdgeListVn2(FILE *fid)
{
    NPRMesh& mesh = getActiveMesh();
    computeSolidVoxel(mesh);

    // Parse points
    int num_pts;
    if (fscanf_s(fid, "%d", &num_pts) == EOF) {
        CONS("Cannot parse number of points.\r\n");
        return false;
    }
    CONS("%d points.\r\n", num_pts);
    Array<NPRVertex*> v_array(num_pts, NULL);
    for (int i=0; i<num_pts; i++) {
        float p[3];
        if (fscanf_s(fid, " P %f %f %f ", &p[0], &p[1], &p[2]) == EOF)
        {
            CONS("Error: Cannot parse points.\r\n");
            return false;
        }
        //CONS("P %f %f %f\r\n", p[0], p[1], p[2]);
        Point3D point(p[0], p[1], p[2]);
        Point3D mapped;
        NPRFace* face = getNearestPoint(point, mesh, mapped);
        if (face == NULL) {
            CONS("Error: cannot find a close match.\r\n");
            return false;
        }
        foreach (ei, face->edges(), NPRFace::Edges) {
            NPREdge* e = *ei;
            if ((e->s()->p() - point).abs() < numeric_limits<float>::epsilon()) {
                v_array[i] = e->s();
                break;
            }
            if ((e->twin()->nextInFace()->t()->p() - point).abs() < numeric_limits<float>::epsilon()) {
                v_array[i] = e->twin()->nextInFace()->t();
                break;
            }
        }
        if (v_array[i] == NULL) {
            CONS("Error: no vertex at position [%f %f %f].\r\n",
                p[0], p[1], p[2]);
            face->color() = Color3d(1.0, 0.0, 0.0);
            face->v0()->marked = true;
            face->v1()->marked = true;
            face->v2()->marked = true;
            return false;
        }
    }

    // Parse feature edges.
    int num_edges;
    if (fscanf_s(fid, "%d", &num_edges) == EOF) {
        CONS("Cannot parse number of edges.\r\n");
        return false;
    }
    CONS("%d edges.\r\n", num_edges);
    for (int i=0; i<num_edges; i++) {
        char type;
        int s_index, t_index;
        if (fscanf_s(fid, " E %1s ", &type) == EOF) {
            CONS("Error: cannot parse edge tyes.\r\n");
            return false;
        }
        if (fscanf_s(fid, " %d %d ", &s_index, &t_index) == EOF) {
            CONS("Error: cannot parse edge.\r\n");
            return false;
        }
        NPREdge* e = v_array[s_index]->getEdgeTo(v_array[t_index]);
        if (e == NULL || e->twin() == NULL) {
            CONS("Error: cannot find edge %d %d.\r\n", s_index, t_index);
            v_array[s_index]->marked = true;
            v_array[t_index]->marked = true;
            return false;
        }
        e->reset();
        e->twin()->reset();
        switch (type) {
            case 'S':
                e->addProp(EP_STRAIGHT);
                e->twin()->addProp(EP_STRAIGHT);
                //e->count = e->twin()->count = 50;
                break;
            case 'R':
                e->addProp(EP_CIRCLE);
                e->twin()->addProp(EP_CIRCLE);
                //e->count = e->twin()->count = 100;
                break;
            case 'C':
                e->addProp(EP_CIRCLE_SEGMENT);
                e->twin()->addProp(EP_CIRCLE_SEGMENT);
                //e->count = e->twin()->count = 140;
                break;
            case 'G':
                e->addProp(EP_GENERAL);
                e->twin()->addProp(EP_GENERAL);
                //e->count = e->twin()->count = 180;
                break;
        }
        e->count = m_drawerB.getMaxTol();
        e->twin()->count = m_drawerB.getMaxTol();
        float n1[3]; float n2[3];
        if (fscanf_s(fid, " N %f %f %f ", &n1[0], &n1[1], &n1[2]) == EOF ||
            fscanf_s(fid, " N %f %f %f ", &n2[0], &n2[1], &n2[2]) == EOF) {
                CONS("Error: cannot parse normal of edge %d %d.\r\n",
                    s_index, t_index);
                return false;
        }
        e->normal = Point3D(n1[0], n1[1], n1[2]);
        e->twin()->normal = Point3D(n2[0], n2[1], n2[2]);
    }

    // Parse virtual edges.
    int num_v_edges;
    if (fscanf_s(fid, "%d", &num_v_edges) == EOF) {
        CONS("Cannot parse number of virtual edges.\r\n");
        return false;
    }
    CONS("%d virtual edges.\r\n", num_v_edges);
    for (int i=0; i<num_v_edges; i++) {
        char type;
        int s_index, t_index;
        if (fscanf_s(fid, " E %1s ", &type) == EOF) {
            CONS("Error: cannot parse edge types.\r\n");
            return false;
        }
        if (fscanf_s(fid, " %d %d ", &s_index, &t_index) == EOF) {
            CONS("Error: cannot parse edge.\r\n");
            return false;
        }
        NPREdge* e = v_array[s_index]->getEdgeTo(v_array[t_index]);
        if (e == NULL || e->twin() == NULL) {
            CONS("Error: cannot find edge %d %d.\r\n", s_index, t_index);
            return false;
        }
        e->reset();
        e->twin()->reset();
        switch (type) {
            case 'S':
                e->addProp(EP_STRAIGHT);
                e->twin()->addProp(EP_STRAIGHT);
                break;
            case 'R':
                e->addProp(EP_CIRCLE);
                e->twin()->addProp(EP_CIRCLE);
                break;
            case 'C':
                e->addProp(EP_CIRCLE_SEGMENT);
                e->twin()->addProp(EP_CIRCLE_SEGMENT);
                break;
            case 'G':
                e->addProp(EP_GENERAL);
                e->twin()->addProp(EP_GENERAL);
                break;
        }
        e->count = 1;
        e->twin()->count = 1;
        float n1[3]; float n2[3];
        if (fscanf_s(fid, " N %f %f %f ", &n1[0], &n1[1], &n1[2]) == EOF ||
            fscanf_s(fid, " N %f %f %f ", &n2[0], &n2[1], &n2[2]) == EOF) {
                CONS("Error: cannot parse normal of edge %d %d.\r\n",
                    s_index, t_index);
                return false;
        }
        e->normal = Point3D(n1[0], n1[1], n1[2]);
        e->twin()->normal = Point3D(n2[0], n2[1], n2[2]);
    }
    restoreAnchors(mesh);
    CONS("Parsing done.\r\n");
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::updateEdgeCount(const Array<EdgeListElement>& edgeList) {
    NPRMesh& mesh = getActiveMesh();
    foreach_const(i, edgeList, Array<EdgeListElement>) {
        foreach (ei, mesh.edges(), NPRMesh::Edges) {
            if ((ei->s()->p() - i->getPoint1()).abs() < numeric_limits<float>::epsilon() &&
                (ei->t()->p() - i->getPoint2()).abs() < numeric_limits<float>::epsilon()) {
                    ei->count = m_drawerB.getMaxTol();
                    if (ei->twin())
                        ei->twin()->count = m_drawerB.getMaxTol();
                    break;
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::updateEdgeNormal(NPRMesh& mesh)
{
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NormalMap::iterator iter =
            m_normalMap.find(std::pair<NPRVertex*, NPRVertex*>(ei->s(), ei->t()));
        if (iter != m_normalMap.end()) {
            ei->normal = iter->second;
        } else {
            continue;
            // Attempt to estimate edges without normal.
            NPREdge* edge_before = ei->prevInFace();
            NPREdge* edge_after = ei->nextInFace();

            NormalMap::iterator iter =
                m_normalMap.find(std::pair<NPRVertex*, NPRVertex*>
                (edge_before->s(), edge_before->t()));
            Point3D normal_before;
            if (iter != m_normalMap.end()) {
                normal_before = iter->second;
            }
            iter =
                m_normalMap.find(std::pair<NPRVertex*, NPRVertex*>
                (edge_after->s(), edge_after->t()));
            Point3D normal_after;
            if (iter != m_normalMap.end()) {
                normal_after = iter->second;
            }

            ei->normal = normal_before+normal_after;
            if (normal_before.abs() < 1.0-epsilon() && normal_after.abs() < 1.0-epsilon()) {
                /*
                CONS("WARNING: no normals available...\r\n");
                CONS("before: %f %f %f\r\n",
                    normal_before.x(), normal_before.y(), normal_before.z());
                CONS("after: %f %f %f\r\n",
                    normal_after.x(), normal_after.y(), normal_after.z());
                ei->s()->marked = true;
                ei->t()->marked = true;
                MB("pause here");
                ei->s()->marked = false;
                ei->t()->marked = false;
                */
            } else {
                ei->normal.normalize();
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::updateVertexNormal(NPRMesh& mesh)
{
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        vi->m_normals.clear();
    }

    foreach (fi, mesh.faces(), NPRMesh::Faces) {
        NPRFace* f = &*fi;
        foreach (ei, f->edges(), NPRFace::Edges) {
            Point3D normal1 = (*ei)->normal;
            Point3D normal2 = (*ei)->prevInFace()->normal;
            (*ei)->s()->m_normals[(*ei)->face()] = (normal1 + normal2).unit();
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::removeIsolatedVertices(NPRMesh& mesh)
{
    mesh.eraseNotConnectedVertices();
    return;
    /*
    Array<NPRVertex*> to_remove;
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        if (vi->nNeighbors() == 0) {
            to_remove.push_back(&*vi);
        }
    }

    foreach (vi, to_remove, Array<NPRVertex*>) {
        mesh.removeVertex(*vi);
    }
    */
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadData(const CString& filename, NPRMesh& mesh)
{
    CONS("Loading core mesh from %s\r\n", filename);
    FILE *fid = 0;
    if (fopen_s(&fid, filename, "r")) {
        TRACE("ERROR: unable to open file %s\r\n", filename);
        return false;
    }
    
    int version=1;
    if (fscanf_s(fid, "version %d ", &version) != 1) {
        CONS("ERROR: no version string at the beginning.\r\n");
        fclose(fid);
        return false;
    }

    bool result = false;
    switch (version) {
        case -2:
            result = loadEdgeListVn2(fid);
            CONS("Done loading edge list (version -2).\r\n");
            break;
        case -1:
            result = loadEdgeListVn1(fid);
            CONS("Done loading edge list (version -1).\r\n");
            break;
        case 1:
            result = loadEdgeList(fid);
            CONS("Done loading edge list (version 1).\r\n");
            break;
        case 2:
            result = loadCoreMeshV2(fid, mesh);
            CONS("Done loading core mesh (version 2).\r\n");
            break;
        case 3:
            result = loadCoreMeshV3(fid, mesh);
            CONS("Done loading core mesh (version 3).\r\n");
            break;
        case 4:
            result = loadCoreMeshV4(fid, mesh);
            CONS("Done loading core mesh (version 4).\r\n");
            break;
        default:
            CONS("Error: unknown version %d.\r\n", version);
            break;
    }

    fclose(fid);
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadCoreMeshV2(FILE* fid, NPRMesh& mesh)
{
    /*
    CONS("Loading core mesh from %s\r\n", filename);
    FILE *fid = 0;
    fid = fopen(filename, "r");
    if (!fid) {
        TRACE("ERROR: unable to open file %s\r\n", filename);
        return false;
    }
    
    int version=1;
    if (fscanf_s(fid, "version %d ", &version) != 1) {
        CONS("ERROR: no version string at the beginning.\r\n");
        fclose(fid);
        return false;
    }

    ASSERT(version == 2);
    */

    int num_v = 0, num_f = 0, num_anchor_edges;
    if (fscanf_s(fid, " %d %d %d ", &num_v, &num_f, &num_anchor_edges) != 3) {
        fclose(fid);
        return false;
    }
    CONS("Number of vertices: %d    Number of faces: %d\r\n", num_v, num_f);

    Array<NPRVertex*> vertices;
    for (int i=0; i<num_v; i++) {
        float x, y, z;
        fscanf_s(fid, " P %f %f %f ", &x, &y, &z);
        NPRVertex* v = mesh.addVertex(Point3D(x, y, z));
        v->m_index = i;
        vertices.push_back(v);
    }

    for (int i=0; i<num_f; i++) {
        int num_vertices = 0; // num vertices of this face
        if (fscanf_s(fid, " F %d ", &num_vertices) != 1) {
            fclose(fid);
            return false;
        }
        List<NPRVertex*> v_list;
        for (int j=0; j<num_vertices; j++) {
            int v_index;
            if (fscanf_s(fid, " %d ", &v_index) != 1) {
                fclose(fid);
                return false;
            }
            v_list.push_back(vertices[v_index]);
        }
        NPRFace* f = mesh.addFace(v_list);

        foreach (vi, v_list, List<NPRVertex*>) {
            float normal[3];
            if (fscanf_s(fid, " N %f %f %f ", &normal[0], &normal[1], &normal[2]) != 3) {
                fclose(fid);
                return false;
            }
            (*vi)->m_normals[f] = Point3D(normal[0], normal[1], normal[2]);
        }
    }

    for (int i=0; i<num_anchor_edges; i++) {
        int v_index1 = 0, v_index2 = 0;
        if (fscanf_s(fid, " E %d %d ", &v_index1, &v_index2) != 2) {
            fclose(fid);
            return false;
        }
        NPREdge* edge = vertices[v_index1]->getEdgeTo(vertices[v_index2]);
        edge->count = m_drawerB.getMaxTol();
        if (edge->twin()) {
            edge->twin()->count = m_drawerB.getMaxTol();
        }
    }

    //fclose(fid);
    //CONS("DONE loading.\r\n");
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadCoreMeshV3(FILE* fid, NPRMesh& mesh)
{
    int num_v = 0, num_f = 0, num_anchor_edges;
    if (fscanf_s(fid, " %d %d %d ", &num_v, &num_f, &num_anchor_edges) != 3) {
        fclose(fid);
        return false;
    }
    CONS("Number of vertices: %d    Number of faces: %d\r\n", num_v, num_f);

    Array<NPRVertex*> vertices;
    for (int i=0; i<num_v; i++) {
        float x, y, z;
        fscanf_s(fid, " P %f %f %f ", &x, &y, &z);
        NPRVertex* v = mesh.addVertex(Point3D(x, y, z));
        v->m_index = i;
        vertices.push_back(v);
    }

    typedef Array<std::pair<NPRVertex*, NPRVertex*> >::iterator eIterator;
    Array<std::pair<NPRVertex*, NPRVertex*> > nonFeatureEdges;
    for (int i=0; i<num_f; i++) {
        int num_vertices = 0; // num vertices of this face
        if (fscanf_s(fid, " F %d ", &num_vertices) != 1) {
            fclose(fid);
            return false;
        }
        List<NPRVertex*> v_list;
        NPRVertex* prev_v = NULL;
        for (int j=0; j<num_vertices; j++) {
            int v_index;
            bool isNonFeature = false;
            if (fscanf_s(fid, " * %d ", &v_index) == 1) {
                // edge is non-feature edge
                isNonFeature = true;
            } else if (fscanf_s(fid, " %d ", &v_index) != 1) {
                fclose(fid);
                return false;
            }
            v_list.push_back(vertices[v_index]);

            // update the non-feature edge list.
            if (prev_v != NULL) {
                nonFeatureEdges.push_back(
                    std::pair<NPRVertex*, NPRVertex*>(prev_v, vertices[v_index]));
                prev_v = NULL;
            }
            if (isNonFeature) {
                prev_v = vertices[v_index];
            }
        }
        if (prev_v != NULL) {
            nonFeatureEdges.push_back(
                std::pair<NPRVertex*, NPRVertex*>(prev_v, v_list.front()));
            prev_v = NULL;
        }
        NPRFace* f = mesh.addFace(v_list);

        foreach (vi, v_list, List<NPRVertex*>) {
            float normal[3];
            if (fscanf_s(fid, " N %f %f %f ", &normal[0], &normal[1], &normal[2]) != 3) {
                fclose(fid);
                return false;
            }
            Point3D n(normal[0], normal[1], normal[2]);
            if (n.abs() < epsilon()) (*vi)->marked = true;
            (*vi)->m_normals[f] = n;//Point3D(normal[0], normal[1], normal[2]);
        }
    }

    // Note: reading the E block is no longer necessary.
    for (int i=0; i<num_anchor_edges; i++) {
        int v_index1 = 0, v_index2 = 0;
        if (fscanf_s(fid, " E %d %d ", &v_index1, &v_index2) != 2) {
            fclose(fid);
            return false;
        }
    }

    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        ei->count = m_drawerB.getMaxTol();
    }

    for (int i=0; i<nonFeatureEdges.size(); i++) {
        std::pair<NPRVertex*, NPRVertex*> p = nonFeatureEdges[i];
        NPREdge* edge = p.first->getEdgeTo(p.second);
        edge->count = 0;
        if (edge->twin()) {
            edge->twin()->count = 0;
        }
    }

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadCoreMeshV4(FILE* fid, NPRMesh& mesh)
{
    int num_v = 0, num_f = 0, num_anchor_edges;
    if (fscanf_s(fid, " %d %d %d ", &num_v, &num_f, &num_anchor_edges) != 3) {
        fclose(fid);
        return false;
    }
    CONS("Number of vertices: %d    Number of faces: %d\r\n", num_v, num_f);

    Array<NPRVertex*> vertices;
    for (int i=0; i<num_v; i++) {
        float x, y, z;
        fscanf_s(fid, " P %f %f %f ", &x, &y, &z);
        NPRVertex* v = mesh.addVertex(Point3D(x, y, z));
        v->m_index = i;
        vertices.push_back(v);
    }

    typedef Array<std::pair<NPRVertex*, NPRVertex*> >::iterator eIterator;
    Array<std::pair<NPRVertex*, NPRVertex*> > nonFeatureEdges;
    for (int i=0; i<num_f; i++) {
        int num_vertices = 0; // num vertices of this face
        if (fscanf_s(fid, " F %d ", &num_vertices) != 1) {
            fclose(fid);
            return false;
        }
        List<NPRVertex*> v_list;
        NPRVertex* prev_v = NULL;
        for (int j=0; j<num_vertices; j++) {
            int v_index;
            bool isNonFeature = false;
            if (fscanf_s(fid, " -1 %d ", &v_index) == 1) {
                // edge is non-feature edge
                isNonFeature = true;
            } else if (fscanf_s(fid, " %d ", &v_index) != 1) {
                fclose(fid);
                return false;
            }
            v_list.push_back(vertices[v_index]);

            // update the non-feature edge list.
            if (prev_v != NULL) {
                nonFeatureEdges.push_back(
                    std::pair<NPRVertex*, NPRVertex*>(prev_v, vertices[v_index]));
                prev_v = NULL;
            }
            if (isNonFeature) {
                prev_v = vertices[v_index];
            }
        }
        if (prev_v != NULL) {
            nonFeatureEdges.push_back(
                std::pair<NPRVertex*, NPRVertex*>(prev_v, v_list.front()));
            prev_v = NULL;
        }
        NPRFace* f = mesh.addFace(v_list);

        foreach (vi, v_list, List<NPRVertex*>) {
            float normal[3];
            if (fscanf_s(fid, " N %f %f %f ", &normal[0], &normal[1], &normal[2]) != 3) {
                fclose(fid);
                return false;
            }
            Point3D n(normal[0], normal[1], normal[2]);
            if (n.abs() < epsilon()) (*vi)->marked = true;
            (*vi)->m_normals[f] = n;//Point3D(normal[0], normal[1], normal[2]);
        }
    }

    // Note: reading the E block is no longer necessary.
    for (int i=0; i<num_anchor_edges; i++) {
        int v_index1 = 0, v_index2 = 0;
        if (fscanf_s(fid, " E %d %d ", &v_index1, &v_index2) != 2) {
            fclose(fid);
            return false;
        }
    }

    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        ei->count = m_drawerB.getMaxTol();
    }

    for (int i=0; i<nonFeatureEdges.size(); i++) {
        std::pair<NPRVertex*, NPRVertex*> p = nonFeatureEdges[i];
        NPREdge* edge = p.first->getEdgeTo(p.second);
        edge->count = 0;
        if (edge->twin()) {
            edge->twin()->count = 0;
        }
    }

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadPoints(FILE* fid, NPRMesh& mesh)
{
    int version;
    if (fscanf_s(fid, " version %d ", &version) != 1) {
        CONS("ERROR: no version string.\r\n");
        return false;
    }

    bool result = false;
    switch (version) {
        case 0:
            result = loadPointsDebug(fid, mesh);
            CONS("Done loading points cloud (version debug).\r\n");
            break;
        case 1:
            result = loadPointsV1(fid, mesh);
            CONS("Done loading point cloud (version 1).\r\n");
            break;
        case 2:
            result = loadPointsV2(fid, mesh);
            CONS("Done loading point cloud (version 2).\r\n");
            break;
        case 3:
            result = loadPointsV3(fid, mesh);
            CONS("Done loading point cloud (version 3).\r\n");
            break;
        case 4:
            result = loadPointsV4(fid, mesh);
            CONS("Done loading point cloud (version 4).\r\n");
            break;
        case 5:
            result = loadPointsV5(fid, mesh);
            CONS("Done loading point cloud (version 5).\r\n");
            break;
        case 6:
            result = loadPointsV6(fid, mesh);
            CONS("Done loading point cloud (version 6).\r\n");
            break;
        default:
            CONS("ERROR: unknown version number %d.\r\n", version);
            break;
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadPointsV1(FILE* fid, NPRMesh& mesh)
{
    int dim_x, dim_y, dim_z;
    if (fscanf_s(fid, " %d %d %d ", &dim_x, &dim_y, &dim_z) != 3) {
        return false;
    }
    CONS("dim: %d %d %d\r\n", dim_x, dim_y, dim_z);

    real max_x = 0.0,
         min_x = bigval(),
         max_y = 0.0,
         min_y = bigval(),
         max_z = 0.0,
         min_z = bigval();
    while (!feof(fid)) {
        float x,y,z,nx,ny,nz;
        if (fscanf_s(fid, " %f %f %f %f %f %f ", &x, &y, &z, &nx, &ny, &nz) != 6) {
            return false;
        }
        if (x > max_x) max_x = x;
        if (x < min_x) min_x = x;
        if (y > max_y) max_y = y;
        if (y < min_y) min_y = y;
        if (z > max_z) max_z = z;
        if (z < min_z) min_z = z;
        Point3D p(x, y, z);
        ///*
        p = p - Point3D(dim_x/2, dim_y/2, dim_z/2);
        p.z() *= -1;
        p = p/(dim_x/2);
        //*/
        Point3D n(nx, ny, nz);
        NPRVertex* v = mesh.addVertex(p);
        //v->normal = n*0.02;
        v->normal = Point3D(0.0, 0.0, 0.0);
        //CONS("%f %f %f %f %f %f\r\n", x, y, z, nx, ny, nz);
    }
    CONS("x(%f, %f)\r\ny(%f %f)\r\nz(%f %f)\r\n",
        min_x, max_x, min_y, max_y, min_z, max_z);


    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadPointsV2(FILE* fid, NPRMesh& mesh)
{
    int min_x, min_y, min_z;
    int max_x, max_y, max_z;
    if (fscanf_s(fid, " %d %d %d %d %d %d ",
        &min_x, &min_y, &min_z, &max_x, &max_y, &max_z) != 6) {
            return false;
    }
    CONS("%d %d %d \r\n%d %d %d\r\n", min_x, min_y, min_z, max_x, max_y, max_z);
    Point3D center((min_x+max_x)*0.5, (min_y+max_y)*0.5, (min_z+max_z)*0.5);
    int dim_x, dim_y, dim_z;
    if (fscanf_s(fid, " %d %d %d ", &dim_x, &dim_y, &dim_z) != 3) {
        return false;
    }
    CONS("dim: %d %d %d\r\n", dim_x, dim_y, dim_z);

    while (!feof(fid)) {
        float x,y,z,nx,ny,nz;
        if (fscanf_s(fid, " %f %f %f %f %f %f ", &x, &y, &z, &nx, &ny, &nz) != 6) {
            return false;
        }
        Point3D p(x, y, z);
        Point3D n(nx, ny, nz);
        NPRVertex* v = mesh.addVertex(p);
        //v->normal = n*0.02;
        v->normal = Point3D(0.0, 0.0, 0.0);
    }

    translate(mesh, center*(-1.0));
    flipZ(mesh);
    Point3D min_mesh, max_mesh;
    m_meshA.getBoundingBox(min_mesh, max_mesh);
    //CONS("%f %f %f\r\n%f %f %f\r\n",
    //    min_mesh.x(), min_mesh.y(), min_mesh.z(),
    //    max_mesh.x(), max_mesh.y(), max_mesh.z());
    scale(mesh, min_mesh.x()/(min_x-center.x()));

    /*
    Point3D min_mesh, max_mesh, center_mesh, size_mesh;
    m_meshA.getBoundingBox(min_mesh, max_mesh);
    center_mesh = (min_mesh + max_mesh)*0.5;
    size_mesh = max_xyz(max_mesh - center_mesh, center_mesh - min_mesh);
    Point3D min_pt, max_pt, center_pt, size_pt;
    mesh.getBoundingBox(min_pt, max_pt);
    center_pt = (min_pt + max_pt) * 0.5;
    size_pt = max_xyz(max_pt - center_pt, center_pt - min_pt);
    real s = min(size_mesh.x()/size_pt.x(), size_mesh.y()/size_pt.y());
    s = min(s, size_mesh.z()/size_pt.z());
    translate(mesh, center_pt*(-1));
    //scale(mesh, s);
    translate(mesh, center_mesh);
    */

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadPointsV3(FILE* fid, NPRMesh& mesh)
{
    int min_x, min_y, min_z;
    int max_x, max_y, max_z;
    if (fscanf_s(fid, " %d %d %d %d %d %d ",
        &min_x, &min_y, &min_z, &max_x, &max_y, &max_z) != 6) {
            return false;
    }
    CONS("%d %d %d \r\n%d %d %d\r\n", min_x, min_y, min_z, max_x, max_y, max_z);
    Point3D center((min_x+max_x)*0.5, (min_y+max_y)*0.5, (min_z+max_z)*0.5);

    int num_strokes;
    if (fscanf_s(fid, " %d ", &num_strokes) != 1) {
        return false;
    }
    for (int i=0; i<num_strokes; i++) {
        m_strokes.push_back();
        Stroke* stroke = &m_strokes.back();

        int num_v;
        if (fscanf_s(fid, " %d ", &num_v) != 1) {
            return false;
        }
        NPRVertex* prev = NULL;
        for (int j=0; j<num_v; j++) {
            float x,y,z;
            if (fscanf_s(fid, " %f %f %f ", &x, &y, &z) != 3) {
                return false;
            }
            NPRVertex* v = mesh.addVertex(Point3D(x,y,z));
            v->normal = Point3D(0.0, 0.0, 0.0);
            SVertex sv(v);
            stroke->push_back(sv);
            if (prev != NULL) prev->next = v;
            prev = v;
        }

        if (stroke->size() < 5) {
            m_strokes.pop_back();
        }
    }

    m_drawerB.setStrokes(&m_strokes);
    CONS("Generated %d strokes.\r\n", m_strokes.size());

    translate(mesh, center*(-1.0));
    flipZ(mesh);
    Point3D min_mesh, max_mesh;
    m_meshA.getBoundingBox(min_mesh, max_mesh);
    //CONS("%f %f %f\r\n%f %f %f\r\n",
    //    min_mesh.x(), min_mesh.y(), min_mesh.z(),
    //    max_mesh.x(), max_mesh.y(), max_mesh.z());
    scale(mesh, min_mesh.x()/(min_x-center.x()));

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadPointsV4(FILE* fid, NPRMesh& mesh)
{
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    if (fscanf_s(fid, " %f %f %f %f %f %f ",
        &min_x, &min_y, &min_z, &max_x, &max_y, &max_z) != 6) {
            return false;
    }
    CONS("%f %f %f \r\n%f %f %f\r\n", min_x, min_y, min_z, max_x, max_y, max_z);
    Point3D center((min_x+max_x)*0.5, (min_y+max_y)*0.5, (min_z+max_z)*0.5);

    int num_strokes;
    if (fscanf_s(fid, " %d ", &num_strokes) != 1) {
        return false;
    }
    for (int i=0; i<num_strokes; i++) {
        m_strokes.push_back();
        Stroke* stroke = &m_strokes.back();

        int num_v;
        if (fscanf_s(fid, " %d ", &num_v) != 1) {
            return false;
        }
        NPRVertex* prev = NULL;
        for (int j=0; j<num_v; j++) {
            float x,y,z;
            if (fscanf_s(fid, " %f %f %f ", &x, &y, &z) != 3) {
                return false;
            }
            NPRVertex* v = mesh.addVertex(Point3D(x,y,z));
            v->normal = Point3D(0.0, 0.0, 0.0);
            SVertex sv(v);
            stroke->push_back(sv);
            if (prev != NULL) prev->next = v;
            prev = v;
        }

        if (stroke->size() < 5) {
            m_strokes.pop_back();
        }
    }

    m_drawerB.setStrokes(&m_strokes);
    CONS("Generated %d strokes.\r\n", m_strokes.size());

    translate(mesh, center*(-1.0));
    flipZ(mesh);
    Point3D min_mesh, max_mesh;
    m_meshA.getBoundingBox(min_mesh, max_mesh);
    //CONS("%f %f %f\r\n%f %f %f\r\n",
    //    min_mesh.x(), min_mesh.y(), min_mesh.z(),
    //    max_mesh.x(), max_mesh.y(), max_mesh.z());
    scale(mesh, min_mesh.x()/(min_x-center.x()));

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadPointsV5(FILE* fid, NPRMesh& mesh)
{
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    if (fscanf_s(fid, " %f %f %f %f %f %f ",
        &min_x, &min_y, &min_z, &max_x, &max_y, &max_z) != 6) {
            return false;
    }
    CONS("%f %f %f \r\n%f %f %f\r\n", min_x, min_y, min_z, max_x, max_y, max_z);
    Point3D center((min_x+max_x)*0.5, (min_y+max_y)*0.5, (min_z+max_z)*0.5);

    int num_vertices;
    if (fscanf_s(fid, " %d ", &num_vertices) != 1) {
        return false;
    }

    Array<SVertex> sv_list(num_vertices);
    Array<bool> used(num_vertices, false);
    Array<int> next_sv(num_vertices, -1);
    Array<int> prev_sv(num_vertices, -1);
    for (int i=0; i<num_vertices; i++) {
        float x,y,z;
        if (fscanf_s(fid, " %f %f %f ", &x, &y, &z) != 3) {
            return false;
        }
        NPRVertex* v = mesh.addVertex(Point3D(x,y,z));
        v->normal = Point3D(0.0, 0.0, 0.0);
        sv_list[i] = SVertex(v);
    }

    int num_strokes;
    if (fscanf_s(fid, " %d ", &num_strokes) != 1) {
        return false;
    }
    for (int i=0; i<num_strokes; i++) {
        m_strokes.push_back();
        Stroke* stroke = &m_strokes.back();

        for (int j=0; j<num_vertices; j++) {
            next_sv[j] = -1;
            prev_sv[j] = -1;
        }
        int num_pair;
        if (fscanf_s(fid, " %d ", &num_pair) != 1) {
            return false;
        }
        int start;
        for (int j=0; j<num_pair; j++) {
            int first, second;
            if (fscanf_s(fid, " %d %d ", &first, &second) != 2) {
                return false;
            }
            if (j == 0) start = first;
            next_sv[first] = second;
            prev_sv[second] = first;
        }
        while (prev_sv[start] != -1)
            start = prev_sv[start];

        while (start != -1) {
            used[start] = true;
            stroke->push_back(sv_list[start]);
            start = next_sv[start];
        }

        if (stroke->size() < m_minStrokeLen) {
            m_strokes.back().valid = false;
            //m_strokes.pop_back();
        }
    }

    for (int i=0; i<num_vertices; i++) {
        if (!used[i]) {
            mesh.removeVertex(sv_list[i].vertex);
        }
    }

    m_drawerB.setStrokes(&m_strokes);
    CONS("Generated %d strokes.\r\n", m_strokes.size());

    translate(mesh, center*(-1.0));
    flipZ(mesh);
    Point3D min_mesh, max_mesh;
    m_meshA.getBoundingBox(min_mesh, max_mesh);
    //CONS("%f %f %f\r\n%f %f %f\r\n",
    //    min_mesh.x(), min_mesh.y(), min_mesh.z(),
    //    max_mesh.x(), max_mesh.y(), max_mesh.z());
    scale(mesh, min_mesh.x()/(min_x-center.x()));

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadPointsV6(FILE* fid, NPRMesh& mesh)
{
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    if (fscanf_s(fid, " %f %f %f %f %f %f ",
        &min_x, &min_y, &min_z, &max_x, &max_y, &max_z) != 6) {
            return false;
    }
    CONS("%f %f %f \r\n%f %f %f\r\n", min_x, min_y, min_z, max_x, max_y, max_z);
    Point3D center((min_x+max_x)*0.5, (min_y+max_y)*0.5, (min_z+max_z)*0.5);

    int num_vertices;
    if (fscanf_s(fid, " %d ", &num_vertices) != 1) {
        return false;
    }

    Array<SVertex> sv_list(num_vertices);
    Array<bool> used(num_vertices, false);
    for (int i=0; i<num_vertices; i++) {
        float x,y,z;
        int weight;
        if (fscanf_s(fid, " %f %f %f %d ", &x, &y, &z, &weight) != 4) {
            return false;
        }
        NPRVertex* v = mesh.addVertex(Point3D(x,y,z));
        v->normal = Point3D(0.0, 0.0, 0.0);
        sv_list[i] = SVertex(v);
    }

    int num_strokes;
    if (fscanf_s(fid, " %d ", &num_strokes) != 1) {
        return false;
    }
    for (int i=0; i<num_strokes; i++) {
        m_strokes.push_back();
        Stroke* stroke = &m_strokes.back();

        int num_pair;
        if (fscanf_s(fid, " %d ", &num_pair) != 1) {
            return false;
        }
        int prev;
        int begin;
        real len = 0.0;
        for (int j=0; j<num_pair; j++) {
            int first, second;
            if (fscanf_s(fid, " %d %d ", &first, &second) != 2) {
                return false;
            }
            if (j == 0) {
                begin = first;
                prev = second;
                stroke->push_back(sv_list[first]);
                stroke->push_back(sv_list[second]);
                used[first] = true;
                used[second] = true;
                continue;
            }
            if (prev != first) return false;
            len += (sv_list[first].vertex->p()
                - sv_list[second].vertex->p()).abs();
            stroke->push_back(sv_list[second]);
            used[second] = true;
            prev = second;
        }

        if (len < m_minStrokeLen) {
            m_strokes.back().valid = false;
            //m_strokes.pop_back();
        }
        if (begin == prev) {
            stroke->circular = true;
        }
    }

    for (int i=0; i<num_vertices; i++) {
        if (!used[i]) {
            mesh.removeVertex(sv_list[i].vertex);
        }
    }

    m_drawerB.setStrokes(&m_strokes);
    CONS("Generated %d strokes.\r\n", m_strokes.size());

    translate(mesh, center*(-1.0));
    flipZ(mesh);
    Point3D min_mesh, max_mesh;
    m_meshA.getBoundingBox(min_mesh, max_mesh);
    scale(mesh, min_mesh.x()/(min_x-center.x()));

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::loadPointsDebug(FILE* fid, NPRMesh& mesh)
{
    double distance_threshold, cluster_threshold;
    if (fscanf_s(fid, " distanceThreshold: %lf ", &distance_threshold) != 1) {
        return false;
    }
    if (fscanf_s(fid, " clusterThreshold: %lf ", &cluster_threshold) != 1) {
        return false;
    }
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    if (fscanf_s(fid, " %f %f %f %f %f %f ",
        &min_x, &min_y, &min_z, &max_x, &max_y, &max_z) != 6) {
            return false;
    }
    CONS("%f %f %f \r\n%f %f %f\r\n", min_x, min_y, min_z, max_x, max_y, max_z);
    Point3D center((min_x+max_x)*0.5, (min_y+max_y)*0.5, (min_z+max_z)*0.5);

    while (!feof(fid)) {
        float x,y,z;
        int num_views;
        if (fscanf_s(fid, " %f %f %f %d ", &x, &y, &z, &num_views) != 4) {
            return false;
        }
        Point3D p(x, y, z);
        NPRVertex* v = mesh.addVertex(p);
        v->normal = Point3D(0.0, 0.0, 0.0);
        v->num_views = num_views;
    }

    translate(mesh, center*(-1.0));
    flipZ(mesh);
    Point3D min_mesh, max_mesh;
    m_meshA.getBoundingBox(min_mesh, max_mesh);
    scale(mesh, min_mesh.x()/(min_x-center.x()));

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::savePointsV6(NPRMesh& mesh) const
{
    FILE* fid;
    fopen_s(&fid, m_path, "w");
    if (!fid) {
        CONS("ERROR: Unable to open file %s.\r\n", m_path);
    }
    fprintf_s(fid, "version 6\r\n");
    Point3D min_mesh, max_mesh;
    mesh.getBoundingBox(min_mesh, max_mesh);
    fprintf_s(fid, "%f %f %f\r\n", max_mesh.x(), max_mesh.y(), max_mesh.z());
    fprintf_s(fid, "%f %f %f\r\n", min_mesh.x(), min_mesh.y(), min_mesh.z());

    List<NPRVertex*> v_list;
    std::map<NPRVertex*, int> v_map;
    int v_count = 0;
    foreach_const (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = const_cast<NPRVertex*>(&*vi);
        NPREdge::circulator dummy;
        int degree = featureDegree(v, dummy);
        if (degree != 0) {
            v_list.push_back(v);
            v_map[v] = v_count;
            v_count++;
        }
    }

    List<List<NPRVertex*> > strokes;
    std::map<NPREdge*, bool> e_map;
    foreach_const (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* e = const_cast<NPREdge*>(&*ei);
        e_map[e] = false;
    }
    foreach_const (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* e = const_cast<NPREdge*>(&*ei);
        if (e_map[e]) continue;
        if (e->count > 0 || (e->twin() && e->twin()->count > 0)) {
            e_map[e] = true;
            if (e->twin()) e_map[e->twin()] = true;
            strokes.push_back();
            List<NPRVertex*>& stroke = strokes.back();
            stroke.push_back(e->s());
            stroke.push_back(e->t());
        }
    }

    fprintf_s(fid, "%d\r\n", v_list.size());
    foreach (vi, v_list, List<NPRVertex*>) {
        NPRVertex* v = *vi;
        Point3D p = v->p();
        fprintf_s(fid, "%f %f %f %d\r\n", -p.x(), -p.y(), p.z(), 1);
    }

    fprintf_s(fid, "%d\r\n", strokes.size());
    foreach (si, strokes, List<List<NPRVertex*> >) {
        List<NPRVertex*>& stroke = *si;
        fprintf_s(fid, "%d\r\n", stroke.size()-1);
        NPRVertex* prev;
        foreach (svi, stroke, List<NPRVertex*>) {
            NPRVertex* v = *svi;
            if (svi == stroke.begin()) {
                prev = v;
                continue;
            }

            int prev_index = v_map[prev];
            int curr_index = v_map[v];
            fprintf_s(fid, "%d %d\r\n", prev_index, curr_index);

            prev = v;
        }
    }
    fclose(fid);
    CONS("Done saving points.\r\n");

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::outputCoreMeshV2(NPRMesh& mesh) const
{
    FILE* fid;
    fopen_s(&fid, m_path, "w");
    if (!fid) {
        CONS("ERROR: Unable to open file %s.\r\n", m_path);
    }
    fprintf(fid, "version 2\r\n");

    int count = 0;
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        vi->m_index = count;
        count++;
    }

    Array<NPREdge*> anchor_edges;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        if (ei->count > m_drawerB.getMinTol()) {
            if (ei->s()->m_index > ei->t()->m_index) {
                anchor_edges.push_back(&*ei);
            }
        }
    }
    fprintf(fid, "%d %d %d\r\n", mesh.nVertices(), mesh.faces().size(), anchor_edges.size());
    foreach_const (vi, mesh.vertices(), NPRMesh::Vertices) {
        fprintf(fid, "P %f %f %f\r\n", vi->p().x(), vi->p().y(), vi->p().z());
    }

    foreach_const (fi, mesh.faces(), NPRMesh::Faces) {
        fprintf(fid, "F %d ", fi->edges().size());
        foreach_const (ei, fi->edges(), NPRFace::Edges) {
            fprintf(fid, "%d ", (*ei)->s()->m_index);
        }
        fprintf(fid, "\r\n");
        foreach_const (ei, fi->edges(), NPRFace::Edges) {
            NPREdge* e = *ei;
            Point3D normal1 = e->normal;
            Point3D normal2 = e->prevInFace()->normal;
            Point3D normal = (normal1 + normal2).unit();
            fprintf(fid, "N %f %f %f\r\n", normal.x(), normal.y(), normal.z());
        }
    }

    foreach_const (ei, anchor_edges, Array<NPREdge*>) {
        fprintf(fid, "E %d %d\r\n", (*ei)->s()->m_index, (*ei)->t()->m_index);
    }
    fclose(fid);

    CONS("Done saving core mesh (version 2)");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::outputCoreMeshV3(NPRMesh& mesh) const
{
    FILE* fid;
    fopen_s(&fid, m_path, "w");
    if (!fid) {
        CONS("ERROR: Unable to open file %s.\r\n", m_path);
    }
    fprintf(fid, "version 3\n");

    int count = 0;
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        vi->m_index = count;
        count++;
    }

    Array<NPREdge*> anchor_edges;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        if (ei->count > m_drawerB.getMinTol()) {
            if (ei->s()->m_index > ei->t()->m_index) {
                anchor_edges.push_back(&*ei);
            }
        }
    }
    fprintf(fid, "%d %d %d\n", mesh.nVertices(), mesh.faces().size(), anchor_edges.size());
    foreach_const (vi, mesh.vertices(), NPRMesh::Vertices) {
        fprintf(fid, "P %f %f %f\n", vi->p().x(), vi->p().y(), vi->p().z());
    }

    foreach_const (fi, mesh.faces(), NPRMesh::Faces) {
        fprintf(fid, "F %d ", fi->edges().size());
        foreach_const (ei, fi->edges(), NPRFace::Edges) {
            if ((*ei)->count <= m_drawerB.getMinTol()) {
                fprintf(fid, "*%d ", (*ei)->s()->m_index);
            } else {
                fprintf(fid, "%d ", (*ei)->s()->m_index);
            }
        }
        fprintf(fid, "\n");
        foreach_const (ei, fi->edges(), NPRFace::Edges) {
            /*
            NPREdge* e = *ei;
            Point3D normal1 = e->normal;
            if (normal1.abs() < 1.0-epsilon()) {
                //CONS("Warning: edge without normal!\r\n");
            }
            Point3D normal2 = e->prevInFace()->normal;
            if (normal2.abs() < 1.0-epsilon()) {
                //CONS("Warning: edge without normal!\r\n");
            }
            Point3D normal = (normal1 + normal2).unit();
            */
            Point3D normal = (*ei)->s()->m_normals[(*ei)->face()];
            fprintf(fid, "N %f %f %f\n", normal.x(), normal.y(), normal.z());
        }
    }

    foreach_const (ei, anchor_edges, Array<NPREdge*>) {
        fprintf(fid, "E %d %d\n", (*ei)->s()->m_index, (*ei)->t()->m_index);
    }
    fclose(fid);

    CONS("Done saving core mesh (version 3)");
}


/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::outputCoreMeshV4(NPRMesh& mesh) const
{
    FILE* fid;
    fopen_s(&fid, m_path, "w");
    if (!fid) {
        CONS("ERROR: Unable to open file %s.\r\n", m_path);
    }
    fprintf(fid, "version 4\n");

    int count = 0;
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        vi->m_index = count;
        count++;
    }

    Array<NPREdge*> anchor_edges;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        if (ei->count > m_drawerB.getMinTol()) {
            if (ei->s()->m_index > ei->t()->m_index) {
                anchor_edges.push_back(&*ei);
            }
        }
    }
    fprintf(fid, "%d %d %d\n", mesh.nVertices(), mesh.faces().size(), anchor_edges.size());
    foreach_const (vi, mesh.vertices(), NPRMesh::Vertices) {
        fprintf(fid, "P %f %f %f\n", vi->p().x(), vi->p().y(), vi->p().z());
    }

    foreach_const (fi, mesh.faces(), NPRMesh::Faces) {
        fprintf(fid, "F %d ", fi->edges().size());
        foreach_const (ei, fi->edges(), NPRFace::Edges) {
            if ((*ei)->count <= m_drawerB.getMinTol()) {
                fprintf(fid, "-1 %d ", (*ei)->s()->m_index);
            } else {
                fprintf(fid, "%d ", (*ei)->s()->m_index);
            }
        }
        fprintf(fid, "\n");
        foreach_const (ei, fi->edges(), NPRFace::Edges) {
            /*
            NPREdge* e = *ei;
            Point3D normal1 = e->normal;
            if (normal1.abs() < 1.0-epsilon()) {
                //CONS("Warning: edge without normal!\r\n");
            }
            Point3D normal2 = e->prevInFace()->normal;
            if (normal2.abs() < 1.0-epsilon()) {
                //CONS("Warning: edge without normal!\r\n");
            }
            Point3D normal = (normal1 + normal2).unit();
            */
            Point3D normal = (*ei)->s()->m_normals[(*ei)->face()];
            fprintf(fid, "N %f %f %f\n", normal.x(), normal.y(), normal.z());
        }
    }

    foreach_const (ei, anchor_edges, Array<NPREdge*>) {
        fprintf(fid, "E %d %d\n", (*ei)->s()->m_index, (*ei)->t()->m_index);
    }
    fclose(fid);

    CONS("Done saving core mesh (version 3)");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::outputCurveV1(NPRMesh& mesh) const
{
    FILE* fid;
    fopen_s(&fid, m_path, "w");
    if (!fid) {
        CONS("ERROR: Unable to open file output.txt.\r\n");
    }
    fprintf(fid, "version 1\r\n");
    foreach_const (ei, mesh.edges(), NPRMesh::Edges) {
        if (ei->count > m_drawerB.getMinTol()) {
            fprintf(fid, "P %f %f %f \r\n",
                ei->s()->p().x(),
                ei->s()->p().y(),
                ei->s()->p().z());
            fprintf(fid, "P %f %f %f \r\n",
                ei->t()->p().x(),
                ei->t()->p().y(),
                ei->t()->p().z());
            Point3D normal1 =
                (ei->t()->p() - ei->s()->p()) ^
                (ei->nextInFace()->t()->p() - ei->s()->p());
            normal1.normalize();
            //Point3D normal1 =
            //    (ei->prevOfS()->t()->p() - ei->s()->p()) ^
            //    (ei->t()->p() - ei->s()->p());
            fprintf(fid, "N %f %f %f \r\n",
                normal1.x(), normal1.y(), normal1.z());
            if (ei->twin() != 0) {
                Point3D normal2 =
                    (ei->twin()->nextInFace()->t()->p() - ei->s()->p()) ^
                    (ei->t()->p() - ei->s()->p());
                normal2.normalize();
                //Point3D normal2 =
                //    (ei->t()->p() - ei->s()->p()) ^
                //    (ei->nextOfS()->t()->p() - ei->s()->p());
                fprintf(fid, "N %f %f %f \r\n",
                    normal2.x(), normal2.y(), normal2.z());
                ei->twin()->count = 0;
            }
        }
    }
    fclose(fid);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::outputCurveVn1(NPRMesh& mesh) const
{
    FILE* fid;
    fopen_s(&fid, m_path, "w");
    if (!fid) {
        CONS("ERROR: Unable to open file output.txt.\r\n");
    }
    fprintf(fid, "version -1\r\n");

    std::vector<int> index_map(mesh.nVertices(), -1);
    std::vector<NPRVertex*> v_array;
    int count = 0;
    foreach_const (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = const_cast<NPRVertex*>(&*vi);
        if (v->isProp(VP_ANCHOR)) {
            index_map[v->m_index] = count;
            v_array.push_back(v);
            count++;
        }
    }
    fprintf_s(fid, "%d\r\n", v_array.size());
    foreach_const (vi, v_array, std::vector<NPRVertex*>) {
        const NPRVertex* v = *vi;
        Point3D p = v->p();
        fprintf_s(fid, "P %f %f %f\r\n", p.x(), p.y(), p.z());
    }

    std::set<NPREdge*> edges;
    foreach_const(ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* e = const_cast<NPREdge*>(&*ei);
        if (e->count >= m_drawerB.getMaxTol()) {
            int s_index = index_map[e->s()->m_index];
            int t_index = index_map[e->t()->m_index];
            if (s_index > t_index) e = e->twin();
            edges.insert(e);
        }
    }
    fprintf_s(fid, "%d\r\n", edges.size());
    foreach_const(ei, edges, std::set<NPREdge*>) {
        const NPREdge* e = *ei;
        int s_index = index_map[e->s()->m_index];
        int t_index = index_map[e->t()->m_index];
        fprintf_s(fid, "E %d %d\r\n", s_index, t_index);
        Point3D n1 = e->face()->m_normal;
        Point3D n2 = e->twin()->face()->m_normal;
        fprintf_s(fid, "N %f %f %f\r\n", n2.x(), n2.y(), n2.z());
        fprintf_s(fid, "N %f %f %f\r\n", n1.x(), n1.y(), n1.z());
    }

    edges.clear();
    foreach_const(ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* e = const_cast<NPREdge*>(&*ei);
        if (e->count > 0 && e->count < m_drawerB.getMaxTol()) {
            int s_index = index_map[e->s()->m_index];
            int t_index = index_map[e->t()->m_index];
            if (s_index > t_index) e = e->twin();
            edges.insert(e);
        }
    }
    fprintf_s(fid, "%d\r\n", edges.size());
    foreach_const(ei, edges, std::set<NPREdge*>) {
        const NPREdge* e = *ei;
        int s_index = index_map[e->s()->m_index];
        int t_index = index_map[e->t()->m_index];
        fprintf_s(fid, "E %d %d\r\n", s_index, t_index);
        Point3D n1 = e->face()->m_normal;
        Point3D n2 = e->twin()->face()->m_normal;
        fprintf_s(fid, "N %f %f %f\r\n", n2.x(), n2.y(), n2.z());
        fprintf_s(fid, "N %f %f %f\r\n", n1.x(), n1.y(), n1.z());
    }

    fclose(fid);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::updateVoxelGrid()
{
    Point3D v_min, v_max;
    foreach (vi, m_meshA.vertices(), NPRMesh::Vertices) {
        v_min.extend_down(vi->p());
        v_max.extend_up(vi->p());
    }

    Point3D dim = v_max - v_min;
    dim /= max(dim.x(), dim.y(), dim.z());
    dim *= m_maxVoxelPerRow;
    dim = max_xyz(dim, Point3D(7,7,7));
    if (int(dim.x())%2 == 0) dim.x() += 1;
    if (int(dim.y())%2 == 0) dim.y() += 1;
    if (int(dim.z())%2 == 0) dim.z() += 1;
    m_vgrid.create(int(dim.x()), int(dim.y()), int(dim.z()));
    m_vgrid.resize(v_min*1.5, v_max*1.5, 0);
    m_vgrid.generateNormals(0);

    Point3D size = v_max*1.5 - v_min*1.5;
    size.x() /= int(dim.x());
    size.y() /= int(dim.y());
    size.z() /= int(dim.z());
    vox_size = max(max(size.x(), size.y()), size.z());
    CONS("Vox size: %f\r\n", vox_size);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::setDisplayVisualHull(bool v)
{
	CriticalSection cs(m_mutex);
    if (v) {
        CONS("Voxel Hull Active!\r\n");
    	m_uiB.setMesh(&m_vHull, &m_mutex);
        m_uiA.setMesh(&m_vHull, &m_mutex);
	    generateNormals(m_vHull, 0, 1);
        m_drawerB.setReferenceMesh(&m_meshA);
        m_vHull.appearanceSensor().modify();
        m_meshA.appearanceSensor().modify();
        m_activeMesh = visualhull;
    } else {
        CONS("Mesh A active!\r\n");
    	m_uiB.setMesh(&m_meshA, &m_mutex);
        m_uiA.setMesh(&m_meshA, &m_mutex);
	    generateNormals(m_meshA, 0, 1);
        m_drawerB.setReferenceMesh(&m_vHull);
        m_meshA.appearanceSensor().modify();
        m_vHull.appearanceSensor().modify();
        m_activeMesh = meshA;
    }
    m_displayVisualHull = v;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::generateVoxelSurface()
{
    m_vHull.clear();

    std::map<VoxGrid::Vertex*, NPRVertex*> vMap;
    foreach (vi, m_vgrid.vertices, VoxGrid::Vertices) {
        NPRVertex* v = m_vHull.addVertex(vi->p[0]);
        vMap[&*vi] = v;
    }

    for (int i=0; i < m_vgrid.ordVoxels.width(); ++i) {
		for (int j=0; j < m_vgrid.ordVoxels.height(); ++j) {
			for (int k=0; k < m_vgrid.ordVoxels.depth(); ++k) {
                Voxel* voxel = m_vgrid.vox(i,j,k);
                if (voxel->confW() > epsilon()) {
                    // check left
                    if (i == 0 || m_vgrid.vox(i-1,j,k)->confW() <= epsilon()) {
                        List<NPRVertex*> vList1;
                        List<NPRVertex*> vList2;
                        vList1.push_back(vMap[voxel->vertices[0]]);
                        vList1.push_back(vMap[voxel->vertices[3]]);
                        vList1.push_back(vMap[voxel->vertices[7]]);
                        vList2.push_back(vMap[voxel->vertices[0]]);
                        vList2.push_back(vMap[voxel->vertices[7]]);
                        vList2.push_back(vMap[voxel->vertices[4]]);
                        if (m_vHull.addFaceCheck(vList1)) {
                            m_vHull.addFace(vList1);
                        }
                        if (m_vHull.addFaceCheck(vList2)) {
                            m_vHull.addFace(vList2);
                        }
                    }
                    // check right
                    if (i == m_vgrid.ordVoxels.width()-1 || m_vgrid.vox(i+1,j,k)->confW() <= epsilon()) {
                        List<NPRVertex*> vList1;
                        List<NPRVertex*> vList2;
                        vList1.push_back(vMap[voxel->vertices[1]]);
                        vList1.push_back(vMap[voxel->vertices[5]]);
                        vList1.push_back(vMap[voxel->vertices[6]]);
                        vList2.push_back(vMap[voxel->vertices[1]]);
                        vList2.push_back(vMap[voxel->vertices[6]]);
                        vList2.push_back(vMap[voxel->vertices[2]]);
                        if (m_vHull.addFaceCheck(vList1)) {
                            m_vHull.addFace(vList1);
                        }
                        if (m_vHull.addFaceCheck(vList2)) {
                            m_vHull.addFace(vList2);
                        }
                    }
                    // check top
                    if (j == m_vgrid.ordVoxels.height()-1 || m_vgrid.vox(i,j+1,k)->confW() <= epsilon()) {
                        List<NPRVertex*> vList1;
                        List<NPRVertex*> vList2;
                        vList1.push_back(vMap[voxel->vertices[3]]);
                        vList1.push_back(vMap[voxel->vertices[2]]);
                        vList1.push_back(vMap[voxel->vertices[6]]);
                        vList2.push_back(vMap[voxel->vertices[3]]);
                        vList2.push_back(vMap[voxel->vertices[6]]);
                        vList2.push_back(vMap[voxel->vertices[7]]);
                        if (m_vHull.addFaceCheck(vList1)) {
                            m_vHull.addFace(vList1);
                        }
                        if (m_vHull.addFaceCheck(vList2)) {
                            m_vHull.addFace(vList2);
                        }
                    }
                    // check buttom
                    if (j == 0 || m_vgrid.vox(i,j-1,k)->confW() <= epsilon()) {
                        List<NPRVertex*> vList1;
                        List<NPRVertex*> vList2;
                        vList1.push_back(vMap[voxel->vertices[0]]);
                        vList1.push_back(vMap[voxel->vertices[4]]);
                        vList1.push_back(vMap[voxel->vertices[5]]);
                        vList2.push_back(vMap[voxel->vertices[0]]);
                        vList2.push_back(vMap[voxel->vertices[5]]);
                        vList2.push_back(vMap[voxel->vertices[1]]);
                        if (m_vHull.addFaceCheck(vList1)) {
                            m_vHull.addFace(vList1);
                        }
                        if (m_vHull.addFaceCheck(vList2)) {
                            m_vHull.addFace(vList2);
                        }
                    }
                    // check front
                    if (k == m_vgrid.ordVoxels.depth()-1 || m_vgrid.vox(i,j,k+1)->confW() <= epsilon()) {
                        List<NPRVertex*> vList1;
                        List<NPRVertex*> vList2;
                        vList1.push_back(vMap[voxel->vertices[0]]);
                        vList1.push_back(vMap[voxel->vertices[1]]);
                        vList1.push_back(vMap[voxel->vertices[2]]);
                        vList2.push_back(vMap[voxel->vertices[0]]);
                        vList2.push_back(vMap[voxel->vertices[2]]);
                        vList2.push_back(vMap[voxel->vertices[3]]);
                        if (m_vHull.addFaceCheck(vList1)) {
                            m_vHull.addFace(vList1);
                        }
                        if (m_vHull.addFaceCheck(vList2)) {
                            m_vHull.addFace(vList2);
                        }
                    }
                    // check back
                    if (k == 0 || m_vgrid.vox(i,j,k-1)->confW() <= epsilon()) {
                        List<NPRVertex*> vList1;
                        List<NPRVertex*> vList2;
                        vList1.push_back(vMap[voxel->vertices[4]]);
                        vList1.push_back(vMap[voxel->vertices[7]]);
                        vList1.push_back(vMap[voxel->vertices[6]]);
                        vList2.push_back(vMap[voxel->vertices[4]]);
                        vList2.push_back(vMap[voxel->vertices[6]]);
                        vList2.push_back(vMap[voxel->vertices[5]]);
                        if (m_vHull.addFaceCheck(vList1)) {
                            m_vHull.addFace(vList1);
                        }
                        if (m_vHull.addFaceCheck(vList2)) {
                            m_vHull.addFace(vList2);
                        }
                    }
                }
            }
		}
	}
    removeIsolatedVertices(m_vHull);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::computeSolidVoxel(NPRMesh& mesh) {
    // Find solid voxels.
    Mapper::init(&mesh, &m_vgrid);
    Mapper::mapVertexWeights(&mesh, &m_vgrid);
    Mapper::mapEdgeWeigths(&mesh, &m_vgrid);
    Mapper::mapFaceWeigths(&mesh, &m_vgrid);
    Mapper::removeHoles(&m_vgrid);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::resolveDiagonalVoxels()
{
    // Start from the iterior because the out most layer will be always empty.
    for (int i=1; i < m_vgrid.ordVoxels.width()-1; ++i) {
		for (int j=1; j < m_vgrid.ordVoxels.height()-1; ++j) {
			for (int k=1; k < m_vgrid.ordVoxels.depth()-1; ++k) {
                Voxel* voxel = m_vgrid.vox(i,j,k);
                if (voxel->confW() <= epsilon()) {
                    if (!resolveEmptyDiagonal(i,j,k))
                        continue; 
                }
                
                resolveVertexDiagonal(i,j,k);
                resolveEdgeDiagonal(i,j,k);
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::resolveEdgeDiagonal(int i, int j, int k)
{
    bool result = false;
    // Edge diagonal
    if (m_vgrid.vox(i+1,j+1,k)->solid == Voxel::SOLID &&
        m_vgrid.vox(i+1,j  ,k)->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j+1,k)->solid != Voxel::SOLID) {
            m_vgrid.vox(i+1,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j+1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i+1,j,k)->confW() = 1.0;
            m_vgrid.vox(i,j+1,k)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i+1,j-1,k)->solid == Voxel::SOLID &&
        m_vgrid.vox(i+1,j  ,k)->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j-1,k)->solid != Voxel::SOLID) {
            m_vgrid.vox(i+1,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j-1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i+1,j,k)->confW() = 1.0;
            m_vgrid.vox(i,j-1,k)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i-1,j+1,k)->solid == Voxel::SOLID &&
        m_vgrid.vox(i-1,j  ,k)->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j+1,k)->solid != Voxel::SOLID) {
            m_vgrid.vox(i-1,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j+1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i-1,j,k)->confW() = 1.0;
            m_vgrid.vox(i,j+1,k)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i-1,j-1,k)->solid == Voxel::SOLID &&
        m_vgrid.vox(i-1,j  ,k)->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j-1,k)->solid != Voxel::SOLID) {
            m_vgrid.vox(i-1,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j-1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i-1,j,k)->confW() = 1.0;
            m_vgrid.vox(i,j-1,k)->confW() = 1.0;
            result = true;
    }

    if (m_vgrid.vox(i+1,j,k+1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i+1,j,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j,k+1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i+1,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k+1)->solid = Voxel::SOLID;
            m_vgrid.vox(i+1,j,k)->confW() = 1.0;
            m_vgrid.vox(i,j,k+1)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i+1,j,k-1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i+1,j,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j,k-1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i+1,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k-1)->solid = Voxel::SOLID;
            m_vgrid.vox(i+1,j,k)->confW() = 1.0;
            m_vgrid.vox(i,j,k-1)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i-1,j,k+1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i-1,j,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j,k+1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i-1,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k+1)->solid = Voxel::SOLID;
            m_vgrid.vox(i-1,j,k)->confW() = 1.0;
            m_vgrid.vox(i,j,k+1)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i-1,j,k-1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i-1,j,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j,k-1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i-1,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k-1)->solid = Voxel::SOLID;
            m_vgrid.vox(i-1,j,k)->confW() = 1.0;
            m_vgrid.vox(i,j,k-1)->confW() = 1.0;
            result = true;
    }

    if (m_vgrid.vox(i,j+1,k+1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i,j+1,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i,j  ,k+1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i,j+1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k+1)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j+1,k)->confW() = 1.0;
            m_vgrid.vox(i,j,k+1)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i,j+1,k-1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i,j+1,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i,j  ,k-1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i,j+1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k-1)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j+1,k)->confW() = 1.0;
            m_vgrid.vox(i,j,k-1)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i,j-1,k+1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i,j-1,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i,j  ,k+1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i,j-1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k+1)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j-1,k)->confW() = 1.0;
            m_vgrid.vox(i,j,k+1)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i,j-1,k-1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i,j-1,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i,j  ,k-1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i,j-1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k-1)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j-1,k)->confW() = 1.0;
            m_vgrid.vox(i,j,k-1)->confW() = 1.0;
            result = true;
    }

    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::resolveVertexDiagonal(int i, int j, int k)
{
    bool result = false;
    // Vertex diagonal
    if (m_vgrid.vox(i+1,j+1,k+1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i+1,j+1,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i+1,j  ,k+1)->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j+1,k+1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i+1,j+1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i+1,j,k+1)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j+1,k+1)->solid = Voxel::SOLID;
            m_vgrid.vox(i+1,j+1,k)->confW() = 1.0;
            m_vgrid.vox(i+1,j,k+1)->confW() = 1.0;
            m_vgrid.vox(i,j+1,k+1)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i+1,j+1,k-1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i+1,j+1,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i+1,j  ,k-1)->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j+1,k-1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i+1,j+1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i+1,j,k-1)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j+1,k-1)->solid = Voxel::SOLID;
            m_vgrid.vox(i+1,j+1,k)->confW() = 1.0;
            m_vgrid.vox(i+1,j,k-1)->confW() = 1.0;
            m_vgrid.vox(i,j+1,k-1)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i+1,j-1,k+1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i+1,j-1,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i+1,j  ,k+1)->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j-1,k+1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i+1,j-1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i+1,j,k+1)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j-1,k+1)->solid = Voxel::SOLID;
            m_vgrid.vox(i+1,j-1,k)->confW() = 1.0;
            m_vgrid.vox(i+1,j,k+1)->confW() = 1.0;
            m_vgrid.vox(i,j-1,k+1)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i+1,j-1,k-1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i+1,j-1,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i+1,j  ,k-1)->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j-1,k-1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i+1,j-1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i+1,j,k-1)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j-1,k-1)->solid = Voxel::SOLID;
            m_vgrid.vox(i+1,j-1,k)->confW() = 1.0;
            m_vgrid.vox(i+1,j,k-1)->confW() = 1.0;
            m_vgrid.vox(i,j-1,k-1)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i-1,j+1,k+1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i-1,j+1,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i-1,j  ,k+1)->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j+1,k+1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i-1,j+1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i-1,j,k+1)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j+1,k+1)->solid = Voxel::SOLID;
            m_vgrid.vox(i-1,j+1,k)->confW() = 1.0;
            m_vgrid.vox(i-1,j,k+1)->confW() = 1.0;
            m_vgrid.vox(i,j+1,k+1)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i-1,j+1,k-1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i-1,j+1,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i-1,j  ,k-1)->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j+1,k-1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i-1,j+1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i-1,j,k-1)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j+1,k-1)->solid = Voxel::SOLID;
            m_vgrid.vox(i-1,j+1,k)->confW() = 1.0;
            m_vgrid.vox(i-1,j,k-1)->confW() = 1.0;
            m_vgrid.vox(i,j+1,k-1)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i-1,j-1,k+1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i-1,j-1,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i-1,j  ,k+1)->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j-1,k+1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i-1,j-1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i-1,j,k+1)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j-1,k+1)->solid = Voxel::SOLID;
            m_vgrid.vox(i-1,j-1,k)->confW() = 1.0;
            m_vgrid.vox(i-1,j,k+1)->confW() = 1.0;
            m_vgrid.vox(i,j-1,k+1)->confW() = 1.0;
            result = true;
    }
    if (m_vgrid.vox(i-1,j-1,k-1)->solid == Voxel::SOLID &&
        m_vgrid.vox(i-1,j-1,k  )->solid != Voxel::SOLID &&
        m_vgrid.vox(i-1,j  ,k-1)->solid != Voxel::SOLID &&
        m_vgrid.vox(i  ,j-1,k-1)->solid != Voxel::SOLID) {
            m_vgrid.vox(i-1,j-1,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i-1,j,k-1)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j-1,k-1)->solid = Voxel::SOLID;
            m_vgrid.vox(i-1,j-1,k)->confW() = 1.0;
            m_vgrid.vox(i-1,j,k-1)->confW() = 1.0;
            m_vgrid.vox(i,j-1,k-1)->confW() = 1.0;
            result = true;
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
// return true if voxel (i,j,k) become solid.
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::resolveEmptyDiagonal(int i, int j, int k)
{
    // Vertex diagonal
    if (m_vgrid.vox(i+1,j+1,k+1)->solid == Voxel::EMPTY &&
        m_vgrid.vox(i+1,j+1,k  )->solid != Voxel::EMPTY &&
        m_vgrid.vox(i+1,j  ,k+1)->solid != Voxel::EMPTY &&
        m_vgrid.vox(i  ,j+1,k+1)->solid != Voxel::EMPTY) {
            m_vgrid.vox(i,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k)->confW() = 1.0;
            return true;
    }
    if (m_vgrid.vox(i+1,j+1,k-1)->solid == Voxel::EMPTY &&
        m_vgrid.vox(i+1,j+1,k  )->solid != Voxel::EMPTY &&
        m_vgrid.vox(i+1,j  ,k-1)->solid != Voxel::EMPTY &&
        m_vgrid.vox(i  ,j+1,k-1)->solid != Voxel::EMPTY) {
            m_vgrid.vox(i,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k)->confW() = 1.0;
            return true;
    }
    if (m_vgrid.vox(i+1,j-1,k+1)->solid == Voxel::EMPTY &&
        m_vgrid.vox(i+1,j-1,k  )->solid != Voxel::EMPTY &&
        m_vgrid.vox(i+1,j  ,k+1)->solid != Voxel::EMPTY &&
        m_vgrid.vox(i  ,j-1,k+1)->solid != Voxel::EMPTY) {
            m_vgrid.vox(i,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k)->confW() = 1.0;
            return true;
    }
    if (m_vgrid.vox(i+1,j-1,k-1)->solid == Voxel::EMPTY &&
        m_vgrid.vox(i+1,j-1,k  )->solid != Voxel::EMPTY &&
        m_vgrid.vox(i+1,j  ,k-1)->solid != Voxel::EMPTY &&
        m_vgrid.vox(i  ,j-1,k-1)->solid != Voxel::EMPTY) {
            m_vgrid.vox(i,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k)->confW() = 1.0;
            return true;
    }
    if (m_vgrid.vox(i-1,j+1,k+1)->solid == Voxel::EMPTY &&
        m_vgrid.vox(i-1,j+1,k  )->solid != Voxel::EMPTY &&
        m_vgrid.vox(i-1,j  ,k+1)->solid != Voxel::EMPTY &&
        m_vgrid.vox(i  ,j+1,k+1)->solid != Voxel::EMPTY) {
            m_vgrid.vox(i,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k)->confW() = 1.0;
            return true;
    }
    if (m_vgrid.vox(i-1,j+1,k-1)->solid == Voxel::EMPTY &&
        m_vgrid.vox(i-1,j+1,k  )->solid != Voxel::EMPTY &&
        m_vgrid.vox(i-1,j  ,k-1)->solid != Voxel::EMPTY &&
        m_vgrid.vox(i  ,j+1,k-1)->solid != Voxel::EMPTY) {
            m_vgrid.vox(i,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k)->confW() = 1.0;
            return true;
    }
    if (m_vgrid.vox(i-1,j-1,k+1)->solid == Voxel::EMPTY &&
        m_vgrid.vox(i-1,j-1,k  )->solid != Voxel::EMPTY &&
        m_vgrid.vox(i-1,j  ,k+1)->solid != Voxel::EMPTY &&
        m_vgrid.vox(i  ,j-1,k+1)->solid != Voxel::EMPTY) {
            m_vgrid.vox(i,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k)->confW() = 1.0;
            return true;
    }
    if (m_vgrid.vox(i-1,j-1,k-1)->solid == Voxel::EMPTY &&
        m_vgrid.vox(i-1,j-1,k  )->solid != Voxel::EMPTY &&
        m_vgrid.vox(i-1,j  ,k-1)->solid != Voxel::EMPTY &&
        m_vgrid.vox(i  ,j-1,k-1)->solid != Voxel::EMPTY) {
            m_vgrid.vox(i,j,k)->solid = Voxel::SOLID;
            m_vgrid.vox(i,j,k)->confW() = 1.0;
            return true;
    }
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

Stroke* CNPRAlg::traceStroke(NPREdge* edge)
{
    m_strokes.push_back();
    Stroke* stroke = &m_strokes.back();

    NPREdge* e = edge;
    //stroke->push_back(SVertex(e->s()));

    // stroke continues in s->t direction
    while (true) {
        NPREdge* next = e;

        SVertex sv(e->t());
        stroke->push_back(sv);
        e->visited = true;
        if (e->twin()) { e->twin()->visited = true; }

        int count = 0;
        NPREdge::circulator ci = e->t()->edge_circulator();
        do {
            if (!(*ci)->visited && (*ci)->count > m_drawerB.getMinTol()) {
                if ((*ci) != e->twin()) {
                    next = *ci;
                    count++;
                }
            }
        } while (++ci != e->t()->edge_circulator() && ci != 0);

        if (count != 1) {
            // End of stroke or T-junction
            break;
        }
        e = next;
    }

    // stroke continues in t->s direction
    if (edge->twin()) {
        e = edge->twin();
        while (true) {
            NPREdge* next = e;

            SVertex sv(e->t());
            stroke->push_front(sv);
            e->visited = true;
            if (e->twin()) { e->twin()->visited = true; }

            int count = 0;
            NPREdge::circulator ci = e->t()->edge_circulator();
            do {
                if (!(*ci)->visited && (*ci)->count > m_drawerB.getMinTol()) {
                    if ((*ci) != e->twin()) {
                        next = *ci;
                        count++;
                    }
                }
            } while (++ci != e->t()->edge_circulator() && ci != 0);

            if (count != 1) {
                // End of stroke or T-junction
                break;
            }
            e = next;
        }
    }

    // check for non-circular strokes
    if ((stroke->front().vertex->p() - stroke->back().vertex->p()).abs() < epsilon()) {
        stroke->circular = true;
        stroke->pop_front();
    } else {
        stroke->circular = false;
    }
    //if (e->t() != edge->s()) {
        //stroke->push_front(SVertex(edge->s()));
        //stroke->circular = false;
    //} else {
        //stroke->circular = true;
    //}

    stroke->reverse();

    return stroke;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::viterbi(NPRMesh& mesh, Stroke* stroke)
{
    SVertex* psv = NULL;
    int count=0;
    foreach(svi, (*stroke), Stroke) {
        SVertex* sv = &*svi;
        if (psv == NULL) {
            /*
            CONS("Initial sv: %d\r\n", sv);
            foreach (si, sv->m_states, SVertex::States) {
                SVertex::State* st = &*si;
                st->delta *= 1e3;
                CONS("delta: %lf\r\n", st->delta);
                st->tv->marked = true;
                MB("Initial look!");
                st->tv->marked = false;
            }
            */
            psv = sv;
            continue;
        }
        if (sv->m_states.empty()) continue;

        int s_index = 0;
        foreach (si, sv->m_states, SVertex::States) {
			SVertex::State* st = &*si;

			viterbi(mesh, st, psv, stroke->debug);
            //CONS("%d:  delta = %10.10f\r\n", s_index, st->delta);
            s_index++;
		}

        count++;
		psv = sv;
    }

	if (psv == NULL) return;

    SVertex::State* mst;

	// trace back the path //////////////////////////
	// find last max delta state ////////////////////
	mst = maxState(psv, stroke->debug);
	if (mst == 0) return;

	// trace back states //////////////////////////////
	traceStates(mst, stroke->debug);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::viterbi(NPRMesh& mesh, SVertex::State* cst, SVertex* psv, bool debug)
{
	// find max delta ///////////////////////
	real mdelta = 0.0;
	SVertex::State* mst = 0;
	//real lp = cst->w;

	foreach (psi, psv->m_states, SVertex::States) {
		SVertex::State* pst = &*psi;
		
		const real slen = (cst->sv->p() - psv->vertex->p()).abs();
		//const real tlen = SVertex::v2vDist(cst->tv, pst->tv);
        //const real tlen = geodesicDist(mesh, cst->tv, pst->tv);
        real tlen;
        if (m_robustMode) {
            // using geodesic distance as metric
            //DistMap::iterator iter =
            //    m_distMap.find(std::pair<NPRVertex*, NPRVertex*>(cst->tv, pst->tv));
            //if (iter != m_distMap.end()) {
            //    tlen = iter->second;
            //} else {
            //    tlen = slen+10.0;
            //}
            //dijkstraReset(&mesh); not necessary because I am using seed.
            real v_dist = (cst->tv->p() - pst->tv->p()).abs();
            /*
            CONS("going in...\r\n");
            CONS("From: %f %f %f\r\n", cst->tv->p().x(), cst->tv->p().y(), cst->tv->p().z());
            CONS("To:   %f %f %f\r\n", pst->tv->p().x(), pst->tv->p().y(), pst->tv->p().z());
            cst->tv->marked = true;
            pst->tv->marked = true;
            MB("debuggin sucks!");
            cst->tv->marked = false;
            pst->tv->marked = false;
            */
            List<NPRFace*> f_list = faceDijkstraRun(mesh, cst->tv, pst->tv, 10*v_dist);
            /*
            foreach (fi, f_list, List<NPRFace*>) {
                (*fi)->color() = Color3d(1.0, 0.0, 0.0);
            }
            MB("check the results...");
            foreach (fi, f_list, List<NPRFace*>) {
                (*fi)->color() = Color3d(1.0, 1.0, 1.0);
            }
            CONS("out!\r\n");
            */
            tlen = 0;
            if (f_list.size() == 0 && cst->tv != pst->tv) {
                tlen = bigval();
            } else {
                Point3D prev = cst->tv->p();
                foreach (fi, f_list, List<NPRFace*>) {
                    NPRFace* face = *fi;
                    Point3D curr = face->centroid();
                    tlen += (curr-prev).abs();
                    prev = curr;
                }
                tlen += (pst->tv->p()-prev).abs();
            }
            /*
            List<NPRVertex*> v_list = vertexDijkstraRun(mesh, cst->tv, pst->tv, 10*v_dist);
            if (v_list.size() == 0 && cst->tv != pst->tv) {
                //CONS("source and target not connected.\r\n");
                tlen = bigval();
            } else {
                tlen = pathDist(v_list);
            }
            */
        } else {
            // using euclidean distance
            tlen = SVertex::v2vDist(cst->tv, pst->tv);
        }

		real tp = epsilon();
		//tp += SVertex::gaussian(tlen, slen, 0.107);
        tp += SVertex::gaussian(tlen, slen, 0.3);
        /*
        if (debug) {
            cst->tv->marked = pst->tv->marked = true;
            CONS("tlen=%f, slen=%f, cost=%10.10f\r\n", tlen, slen, tp);
            MB("think more");
            cst->tv->marked = pst->tv->marked = false;
        }
        */

		real delta = pst->delta * tp;

		if (delta < mdelta) continue;
		mdelta = delta;
		mst = pst;
	}
	ASSERT(mst); 

	cst->prev = mst;
	cst->delta = mdelta * cst->emission_prob;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

SVertex::State* CNPRAlg::maxState(SVertex* lsv, bool debug)
{
	real mdelta = -epsilon(); //0.0;
    SVertex::State* mst = 0;

    //CONS("Last sv: %d\r\n", lsv);
    CONS("Last vertex has %d states.\r\n", lsv->m_states.size());
	foreach (si, lsv->m_states, SVertex::States) {
		SVertex::State* st = &*si;
		real delta = st->delta;

        /*
        if (debug) {
            CONS("delta: %3.10f\r\n", delta);
            st->tv->marked = true;
            MB("Look!");
            st->tv->marked = false;
        }
        */
		if (delta > mdelta) {
			mdelta = delta;
			mst = st;
		}
	}

    CONS("Max delta: %3.10f\r\n", mdelta);
	return mst;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::traceStates(SVertex::State* mst, bool debug)
{
    SVertex::State* cst = mst; // current state

	while (cst) {
		NPRVertex* tv = cst->tv; // vertex on voxel hull
		NPRVertex* sv = cst->sv; // vertex on stroke
        /*
        if (debug) {
            CONS(":delta = %lf\r\n", cst->delta);
            if (cst->delta < 1.0) {
                CONS(" %d states\r\n", cst->parent->m_states.size());
                foreach (sti, cst->parent->m_states, SVertex::States) {
                    CONS(" delta = %lf\r\n", sti->delta);
                }
            }
        }
        */

        if (!tv->isProp(VP_ANCHOR)) {
            cst->parent->mapped_vertex = tv;
            tv->addProp(VP_ANCHOR);
        } else {
            // more than 1 svertex mapped to voxel vertex
            //cst->parent->mapped_vertex = NULL;
            cst->parent->mapped_vertex = tv;
        }

        cst = cst->prev;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::computeLaplacianCoord(NPRMesh& mesh)
{
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = &*vi;
        v->lCoord = Point3D(0.0, 0.0, 0.0);

        real total_weight = 0.0;
        NPREdge::circulator ci = v->edge_circulator();
        do {
            NPREdge* edge = *ci;
            //real w = edge->cotanWeight();
            real w = 1.0;
            v->lCoord += edge->t()->p() * w;
            total_weight += w;
        } while (++ci != v->edge_circulator() && ci != 0);

        v->lCoord /= total_weight;
        v->lCoord -= v->p();
        if (isnan(v->lCoord.x()) ||
            isnan(v->lCoord.y()) ||
            isnan(v->lCoord.z())) {
                CONS("Laplacian coord of Vertex %d is nan: %f %f %f\r\n",
                    v->m_index, v->lCoord.x(), v->lCoord.y(), v->lCoord.z());
                v->marked = true;
                CONS(" p: %f %f %f\r\n", v->p().x(), v->p().y(), v->p().z());
                CONS(" num neighbors: %d\r\n", v->nNeighbors());
                ci = v->edge_circulator();
                do {
                    CONS(" neighbor: %f %f %f\r\n",
                        (*ci)->t()->p().x(),
                        (*ci)->t()->p().y(),
                        (*ci)->t()->p().z());
                } while (++ci != v->edge_circulator() && ci != 0);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::deform(NPRMesh& mesh)
{
    assignVertexIndex(mesh);
    //foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
    //    vi->reset();
    //}

    // Each stroke should form a path on the voxel shell
    // Normal information should be set
    // m_meshA should be replaced by m_vHull
    //int num_anchors = interpolateStrokeOnVoxelShell();

    //computeMeshMapping(mesh, m_meshA);
    //LaplacianSolver solver(mesh.nVertices()+num_anchors, mesh.nVertices());
    int num_constrains = 0;
    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        num_constrains += stroke->size();
    }
    LaplacianSolver solver(mesh.nVertices()+num_constrains, mesh.nVertices());

    //foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
    //    if (vi->isProp(VP_ANCHOR)) {
            //vi->p() = vi->secondPos;
    //    }
    //}
    /*
    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        foreach (svi, (*stroke), Stroke) {
            SVertex* sv = &*svi;
            if (sv->mapped_face == NULL) continue;
            sv->mapped_face->v0()->addProp(VP_ANCHOR);
            sv->mapped_face->v1()->addProp(VP_ANCHOR);
            sv->mapped_face->v2()->addProp(VP_ANCHOR);
        }
    }
    */
    solver.initialize(&mesh);
    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        foreach (svi, (*stroke), Stroke) {
            SVertex* sv = &*svi;
            if (sv->mapped_face == NULL) continue;
            solver.begin_row();
            Point3D b_coor = barycentricCoords(
                sv->mapped_point,
                sv->mapped_face->v0()->p(),
                sv->mapped_face->v1()->p(),
                sv->mapped_face->v2()->p());
            solver.add(sv->mapped_face->v0()->m_index,
                b_coor.x()*LaplacianSolver::w_target);
            solver.add(sv->mapped_face->v1()->m_index,
                b_coor.y()*LaplacianSolver::w_target);
            solver.add(sv->mapped_face->v2()->m_index,
                b_coor.z()*LaplacianSolver::w_target);
            solver.set_rhs(
                sv->vertex->p().x()*LaplacianSolver::w_target,
                sv->vertex->p().y()*LaplacianSolver::w_target,
                sv->vertex->p().z()*LaplacianSolver::w_target);
            solver.end_row();
        }
    }
    solver.solve();
    /*
    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        foreach (svi, (*stroke), Stroke) {
            SVertex* sv = &*svi;
            if (sv->mapped_face == NULL) continue;
            sv->mapped_face->v0()->addProp(VP_ANCHOR);
            sv->mapped_face->v1()->addProp(VP_ANCHOR);
            sv->mapped_face->v2()->addProp(VP_ANCHOR);
        }
    }
    */
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::deformNoAnchor(NPRMesh& mesh)
{
    assignVertexIndex(mesh);
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        vi->reset();
    }
    LaplacianSolver solver(mesh.nVertices()*3, mesh.nVertices());
    LaplacianSolver::w_scale_factor = 0.9;
    solver.initialize(&mesh);
    // add constraints
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        //if (vi->nearestPoint.abs() < epsilon()) continue;
        real soft_weight = epsilon();
        real mean_error = LaplacianSolver::getMeanError();
        //soft_weight = 1.0 - (vi->p()-vi->nearestPoint).abs()/(2*mean_error);
        //soft_weight = max(soft_weight, 0.0);
        /*
        real dist = (vi->p()-vi->nearestPoint).abs();
        soft_weight = gaussian(dist, 0, vox_size);
        soft_weight *= LaplacianSolver::w_target;
        */

        // move to mapped position
        if (vi->nearestPoint.abs() > epsilon()) {
            // Exists a valid mapped position.
            vi->marked = false;
            real dist = (vi->p()-vi->nearestPoint).abs();
            soft_weight = gaussian(dist, 0, m_tol);
            soft_weight *= (0.75*LaplacianSolver::w_target);
            soft_weight += 0.25*LaplacianSolver::w_target;

            solver.begin_row();
            solver.add(vi->m_index, soft_weight);
            solver.set_rhs(
                vi->nearestPoint.x()*soft_weight,
                vi->nearestPoint.y()*soft_weight,
                vi->nearestPoint.z()*soft_weight);
            solver.end_row();
        } else {
            // No mapped position.
            if (vi->degree() == 0) continue;
            soft_weight = 0.25*LaplacianSolver::w_target;
            //soft_weight *= 10;
            vi->marked = true;
            //CONS("non-mapped vertex: (%f, %f, %f).\r\n",
            //    vi->p().x(), vi->p().y(), vi->p().z());
            real degree = real(vi->degree());
            solver.begin_row();
            solver.add(vi->m_index, -1.0 * soft_weight);
            NPREdge::circulator ci = vi->edge_circulator();
            do {
                NPREdge* edge = *ci;
                solver.add(edge->t()->m_index, 1.0/degree * soft_weight);
            } while (++ci != vi->edge_circulator() && ci != 0);
            solver.set_rhs(0.0, 0.0, 0.0);
            solver.end_row();
        }

        // stay at current position
        solver.begin_row();
        solver.add(vi->m_index, LaplacianSolver::w_source);
        solver.set_rhs(
            vi->p().x()*LaplacianSolver::w_source,
            vi->p().y()*LaplacianSolver::w_source,
            vi->p().z()*LaplacianSolver::w_source);
        solver.end_row();
    }
    time_t t = time(NULL);
    solver.solve();
    t = time(NULL) - t;
    CONS("Solved in %d seconds.\r\n", t);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::deformAnchor(NPRMesh& mesh)
{
    assignVertexIndex(mesh);
    LaplacianSolver solver(mesh.nVertices()*2, mesh.nVertices());
    solver.initialize(&mesh);
    computeDistToFeature(mesh);
    // add constraints
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        if (vi->isProp(VP_ANCHOR)) {
            solver.begin_row();
            solver.add(vi->m_index, LaplacianSolver::weight);
            solver.set_rhs(
                vi->secondPos.x()*LaplacianSolver::weight,
                vi->secondPos.y()*LaplacianSolver::weight,
                vi->secondPos.z()*LaplacianSolver::weight);
            if (isnan(vi->secondPos.x())
                || isnan(vi->secondPos.y())
                || isnan(vi->secondPos.z())) {
                    CONS("Error: second pos is bad: %f %f %f\r\n",
                        vi->secondPos.x(), vi->secondPos.y(), vi->secondPos.z());
            }
            solver.end_row();
        } else {
            if (vi->nearestPoint.abs() < epsilon()) {
                continue;
            }
            real soft_weight = epsilon();
            real mean_error = LaplacianSolver::getMeanError();
            soft_weight = 1.0 - (vi->p()-vi->nearestPoint).abs()/(2*mean_error);
            soft_weight = max(soft_weight, 0.0);
            soft_weight *= LaplacianSolver::w_target;
            /*
            NPREdge::circulator ci = vi->edge_circulator();
            do {
                NPRFace* f = (*ci)->face();
                if (f->m_dist < 0.02) {
                    soft_weight = 0;
                    break;
                }
            } while (++ci != vi->edge_circulator() && ci != 0);
            */

            solver.begin_row();
            solver.add(vi->m_index, soft_weight);
            solver.set_rhs(
                vi->nearestPoint.x()*soft_weight,
                vi->nearestPoint.y()*soft_weight,
                vi->nearestPoint.z()*soft_weight);
            if (isnan(vi->nearestPoint.x())
                || isnan(vi->nearestPoint.y())
                || isnan(vi->nearestPoint.z())) {
                    CONS("Error: nearest pt is bad: %f %f %f\r\n",
                        vi->nearestPoint.x(), vi->nearestPoint.y(), vi->nearestPoint.z());
            }
            solver.end_row();
        }
    }
    solver.solve();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

int CNPRAlg::interpolateStrokeOnVoxelShell()
{
    int num_anchors = 0;
    for (int i=0; i<m_strokes.size(); i++) {
        Stroke stroke = interpolateStroke(m_strokes.front());
        num_anchors += stroke.size();
        m_strokes.push_back(stroke);
        m_strokes.pop_front();
    }
    return num_anchors;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

Stroke CNPRAlg::interpolateStroke(Stroke& stroke)
{
    Stroke result;

    //CONS("%d\r\n", stroke.size());
    SVertex* prev = NULL;
    foreach (svi, stroke, Stroke) {
        Stroke::iterator next = svi;
        //int counter = 0;
        //while (next != stroke.end() && next->mapped_vertex == svi->mapped_vertex) {
        //    next++;
        //    counter++;
        //    if (counter % 2 == 0) svi++;
        //}
        //next--;
        SVertex* sv = &*svi;
        //svi = next;

        if (sv->mapped_vertex == stroke.begin()->mapped_vertex) {
            prev = sv;
            result.push_back(stroke.front());
            if (result.back().mapped_vertex == NULL) {
                CONS("ERROR: unmapped vertex (%f %f %f).\r\n",
                    result.back().vertex->p().x(),
                    result.back().vertex->p().y(),
                    result.back().vertex->p().z());
            }
            ASSERT(result.back().mapped_vertex != NULL);
            //result.back().m_si = (--result.end());
            result.back().mapped_vertex->secondPos = stroke.front().vertex->p();
            //result.back().mapped_vertex->p() = stroke.front().vertex->p();
            //result.back().mapped_vertex->marked = true;
            result.back().mapped_vertex->addProp(VP_ANCHOR);
            continue;
        }

        if (sv->mapped_vertex == NULL) continue;

        /*
        Point3D normal1, normal2;
        NormalMap::iterator iter =
            m_normalMap.find(std::pair<NPRVertex*, NPRVertex*>(prev->vertex, sv->vertex));
        if (iter != m_normalMap.end()) {
            normal1 = iter->second;
        } else {
            //CONS("WARNING: normals missing on a stroke segment.\r\n");
        }
        iter = m_normalMap.find(std::pair<NPRVertex*, NPRVertex*>(sv->vertex, prev->vertex));
        if (iter != m_normalMap.end()) {
            normal2 = iter->second;
        } else {
            //CONS("WARNING: normals missing on a stroke segment.\r\n");
        }
        */

        //Point3D normal1 = m_normalMap
        //    [std::pair<NPRVertex*, NPRVertex*>(prev->vertex, sv->vertex)];
        //Point3D normal2 = m_normalMap
        //    [std::pair<NPRVertex*, NPRVertex*>(sv->vertex, prev->vertex)];

        List<SVertex*> temp;
        interpolateTo(result, sv, temp);

        // Update the mapped positions.
        int count = 0;
        int len = temp.size();
        SVertex* sprev = prev;
        foreach (si, temp, List<SVertex*>) {
            SVertex* s = *si;
            s->mapped_vertex->secondPos =
                prev->vertex->p() * real(len-count-1)/real(len) +
                sv->vertex->p() * real(count+1)/real(len);
            //s->mapped_vertex->p() =
            //    prev->vertex->p() * real(len-count-1)/real(len) +
            //    sv->vertex->p() * real(count+1)/real(len);
            count++;

            /*
            // update normal map
            NPREdge* e = sprev->mapped_vertex->getEdgeTo(s->mapped_vertex);
            e->normal = normal1;
            m_normalMap[std::pair<NPRVertex*, NPRVertex*>
                (sprev->mapped_vertex, s->mapped_vertex)] = normal1;
            if (e->twin()) {
                e->twin()->normal = normal2;
                m_normalMap[std::pair<NPRVertex*, NPRVertex*>
                    (s->mapped_vertex, sprev->mapped_vertex)] = normal2;
            }
            */
            sprev = s;
        }

        prev = sv;
    }
    //CONS("2\r\n");

    // Close the loop if circular
    if (stroke.circular) {
        /*
        Point3D normal1, normal2;
        NormalMap::iterator iter =
            m_normalMap.find(std::pair<NPRVertex*, NPRVertex*>
            (stroke.back().vertex, stroke.front().vertex));
        if (iter != m_normalMap.end()) {
            normal1 = iter->second;
        } else {
            //CONS("WARNING: normal missing on a stroke segment.\r\n");
        }
        iter = m_normalMap.find(std::pair<NPRVertex*, NPRVertex*>
            (stroke.front().vertex, stroke.back().vertex));
        if (iter != m_normalMap.end()) {
            normal2 = iter->second;
        } else {
            //CONS("WARNING: normal missing on a stroke segment.\r\n");
        }
        */

        //Point3D normal1 = m_normalMap
        //    [std::pair<NPRVertex*, NPRVertex*>(stroke.back().vertex, stroke.front().vertex)];
        //Point3D normal2 = m_normalMap
        //    [std::pair<NPRVertex*, NPRVertex*>(stroke.front().vertex, stroke.back().vertex)];

        List<SVertex*> temp;
        interpolateTo(result, &stroke.front(), temp);

        // Update the mapped positions.
        int count = 0;
        int len = temp.size();
        SVertex* sprev = &stroke.back();
        foreach (si, temp, List<SVertex*>) {
            SVertex* s = *si;
            s->mapped_vertex->secondPos =
                stroke.back().vertex->p() * real(len-count-1)/real(len) +
                stroke.front().vertex->p() * real(count+1)/real(len);
            //s->mapped_vertex->p() =
            //    stroke.back().vertex->p() * real(len-count-1)/real(len) +
            //    stroke.front().vertex->p() * real(count+1)/real(len);
            count++;

            /*
            // update normal map
            NPREdge* e = sprev->mapped_vertex->getEdgeTo(s->mapped_vertex);
            e->normal = normal1;
            m_normalMap[std::pair<NPRVertex*, NPRVertex*>
                (sprev->mapped_vertex, s->mapped_vertex)] = normal1;
            if (e->twin()) {
                e->twin()->normal = normal2;
                m_normalMap[std::pair<NPRVertex*, NPRVertex*>
                    (s->mapped_vertex, sprev->mapped_vertex)] = normal2;
            }
            */
            sprev = s;
        }
    }

    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Extend the result stroke to sv so that the vertices on voxel shell form a path.
// All the newly added vertices are stored in temp for easier post process.
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::interpolateTo(Stroke& result, SVertex* sv, List<SVertex*>& temp)
{
    NPRMesh& mesh = getActiveMesh();
    //CONS("Interpolate\r\n");
    NPRVertex* cur = result.back().mapped_vertex;
    NPRVertex* target = sv->mapped_vertex;
    if (cur == target) return;

    ASSERT(target != NULL);

    dijkstraReset(&mesh);
    List<NPRVertex*> v_list = vertexDijkstraRun(mesh, cur, target);
    if (v_list.size() == 0) {
        // Second chance, the mesh has been modified.
        CONS("Second Chance!\r\n");
        dijkstraReset(&mesh);
        v_list = vertexDijkstraRun(mesh, cur, target);
        if (v_list.size() == 0 || v_list.size() > 10) {
            cur->marked = true;
            target->marked = true;
            CONS("ERROR: NO way to interpolate strokes without intersecting other strokes.\r\n");
            ASSERT(false);
        }
    }
    foreach (vi, v_list, List<NPRVertex*>) {
        if (*vi == cur) continue;
        result.push_back();
        SVertex* last = &(result.back());
        last->mapped_vertex = *vi;
        //last->mapped_vertex->marked = true;
        last->mapped_vertex->addProp(VP_ANCHOR);
        temp.push_back(last);
    }
    return;

    /*
    while (cur != target) {
        real min_dist = bigval();
        NPRVertex* min_vertex = NULL;
        NPREdge::circulator ci = cur->edge_circulator();
        do {
            NPRVertex* vn = (*ci)->t();
            if (vn->isProp(VP_ANCHOR)) continue;

            real dist = (vn->p() - target->p()).abs();
            if (dist < min_dist) {
                min_dist = dist;
                min_vertex = vn;
            }
        } while (++ci != cur->edge_circulator() && ci != 0);

        if (min_vertex == NULL) {
            CONS("bad!!\r\n");
        }
        ASSERT(min_vertex != NULL);

        result.push_back();
        SVertex* last = &(result.back());
        last->mapped_vertex = min_vertex;
        //last->m_si = result.end();
        last->mapped_vertex->marked = true;
        last->mapped_vertex->addProp(VP_ANCHOR);
        temp.push_back(last);
        cur = min_vertex;
        CONS("cur = %i\r\n", cur);
    }
    CONS("done\r\n");
    */
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::smooth(NPRMesh* mesh, int iteration)
{
    CriticalSection cs(m_mutex);
    for (int i=0; i<iteration; i++) {
        Array<Point3D> positions(mesh->vertices().size());
        int count = 0;
        foreach (vi, mesh->vertices(), NPRMesh::Vertices) {
            positions[count] = vi->p();
            vi->m_index = count;
            count++;
        }

        count = 0;
        foreach (vi, mesh->vertices(), NPRMesh::Vertices) {
            if (vi->isProp(VP_ANCHOR)) continue;
            Point3D new_pos;
            real area_sum = 0.0;
            
            NPREdge::circulator ci = vi->edge_circulator();
	        do {
                NPREdge* edge = *ci;
                real area = edge->face()->area();
                new_pos += positions[edge->t()->m_index] * area;
                area_sum += area;
	        } while (++ci != vi->edge_circulator() && ci != 0);
            new_pos /= area_sum;
            vi->p() = new_pos;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::dijkstraReset(NPRMesh* mesh)
{
    foreach (vi, mesh->vertices(), NPRMesh::Vertices) {
        vi->dist = bigval();
        vi->prev = NULL;
        vi->seed = 0;
    }

    foreach (fi, mesh->faces(), NPRMesh::Faces) {
        fi->m_ring = 0;
        fi->m_dist = 0;
        fi->m_prev = NULL;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// if target == NULL, compute shortest geodesic dist to all vertices.
// The shortest path returned contains no anchor vertices.
/////////////////////////////////////////////////////////////////////////////////////////

List<NPRVertex*> CNPRAlg::vertexDijkstraRun(NPRMesh& mesh, NPRVertex* source, NPRVertex* target, real radius)
{
    unsigned int seed = m_seedCount;
    m_seedCount++;
    if (m_seedCount == 0) { dijkstraReset(&mesh); m_seedCount++; }

    List<NPRVertex*> result;
    Heap<real, NPRVertex*, greater<real>> Q;
    //List<NPRVertex*> Q;
    source->dist = 0.0;
    source->seed = seed;
    source->prev = NULL;
    Q.push(source->dist, source);

    while (!Q.empty()) {
        NPRVertex* cur_vertex = Q.top_value();
        real cur_dist = cur_vertex->dist;
        Q.pop();

        if (cur_vertex == target) {
            break;
        }

        NPREdge::circulator ci = cur_vertex->edge_circulator();
        do {
            NPRVertex* v = (*ci)->t();
            if (v->isProp(VP_ANCHOR) && v != target) continue;
            real dist = (cur_vertex->p() - v->p()).abs();
            if ((v->seed != seed || cur_dist + dist < v->dist)
                && cur_dist + dist < radius) {
                    v->dist = cur_vertex->dist + dist;
                    v->prev = cur_vertex;
                    v->seed = seed;
                    Q.push(v->dist, v);
            }
            //CONS("%f %f %f\r\n", v->p().x(), v->p().y(), v->p().z());
        } while (++ci != cur_vertex->edge_circulator() && ci != 0);
    }

    if (target == NULL) return result;

    NPRVertex* cur_v = target;
    while (cur_v != source && cur_v->seed == seed) {
        result.push_back(cur_v);
        cur_v = cur_v->prev;
    }
    if (cur_v == source) {
        result.push_back(cur_v);
    } else {
        //CONS("Error: unable to interpolate stroke from (%f %f %f) to vertex (%f %f %f) %d\r\n",
        //    source->p().x(), source->p().y(), source->p().z(),
        //    target->p().x(), target->p().y(), target->p().z(),
        //    result.size());
        //source->marked = true;
        //target->marked = true;
        //ASSERT(false);

        /*
        // split certain edges to save the situation.
        CONS("Spliting edges.\r\n");
        List<NPRFace*> f_list = faceDijkstraRun(source, target);
        NPRFace* prev_f = f_list.front();
        foreach (fi, f_list, List<NPRFace*>) {
            if (fi == f_list.begin()) continue;

            NPRFace* f = *fi;
            NPREdge* e = NULL;
            NPRFace::Edges e_list = f->edges();
            foreach (ei, e_list, NPRFace::Edges) {
                NPREdge* edge = *ei;
                if (edge->twin() && edge->twin()->face() == prev_f) {
                    e = edge;
                    break;
                }
            }

            if (e != NULL &&
                e->s()->isProp(VP_ANCHOR) &&
                e->t()->isProp(VP_ANCHOR)) {
                    Point3D new_p = e->s()->p()*0.5 + e->t()->p()*0.5;
                    NPRVertex* new_v = mesh.addVertex(new_p);
                    mesh.refineEdge(e, new_v);
            }

            //CONS("ring: %d\r\n", f->m_ring);
            f->color() = Color3d(0.0, 0.0, 1.0);
            prev_f = f;
        }
        */
    }
    result.reverse();

    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Return a path from source to any anchor in the direction (+- 30) of dir
/////////////////////////////////////////////////////////////////////////////////////////

List<NPRVertex*> CNPRAlg::vertexDijkstraAnchor(NPRVertex *source, real tol, Point3D dir)
{
    List<NPRVertex*> result;
    Heap<real, NPRVertex*, greater<real>> Q;
    //List<NPRVertex*> Q;
    source->dist = 0.0;
    Q.push(source->dist, source);

    NPRVertex* target = NULL;
    real min_dist = bigval();
    while (!Q.empty()) {
        NPRVertex* cur_vertex = Q.top_value();
        real cur_dist = cur_vertex->dist;
        Q.pop();

        //if (cur_vertex->isProp(VP_ANCHOR)
        //    && cur_vertex != source
        //    && (cur_vertex->stroke_index != source->stroke_index
        //    || abs(cur_vertex->sv_index - source->sv_index) > 20)) {
        if (cur_vertex->isProp(VP_ANCHOR) && cur_vertex != source) {
                //real dist = (cur_vertex->p() - source->p()).abs();
                //real dir_offset = 2-dir*(cur_vertex->p() - cur_vertex->prev->p()).unit();
                //if (dir_offset > 1.8) dir_offset *= 10;
                //dist *= dir_offset;
                real dist = cur_vertex->dist;
                if (dist < min_dist) {
                    target = cur_vertex;
                    min_dist = dist;
                }
                continue;
        }

        NPREdge::circulator ci = cur_vertex->edge_circulator();
        do {
            NPRVertex* v = (*ci)->t();
            real dist = (cur_vertex->p() - v->p()).abs();
            //real dist_ratio = 2-dir*(v->p() - cur_vertex->p()).unit();
            real angle = acos(dir*(v->p() - source->p()).unit())*180/M_PI;
            if (angle > 30) continue;
            //if (dist_ratio > 1.8) dist_ratio *= 10;
            //dist *= (dist_ratio*dist_ratio);
            if (cur_dist + dist < v->dist && cur_dist + dist < tol) {
                v->dist = cur_dist + dist;
                v->prev = cur_vertex;
                Q.push(v->dist, v);
            }
        } while (++ci != cur_vertex->edge_circulator() && ci != 0);
    }

    if (target == NULL) return result;
    NPRVertex* cur_v = target;
    while (cur_v->prev != NULL) {
        result.push_back(cur_v);
        cur_v = cur_v->prev;
    }
    if (cur_v == source) {
        //result.push_back(cur_v);
    } else {
        CONS("ERROR: source and target not connected!\r\n");
    }
    result.reverse();

    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Return a path from source to any anchor in any direction.
/////////////////////////////////////////////////////////////////////////////////////////

List<NPRVertex*> CNPRAlg::vertexDijkstraAnchor(NPRVertex *source, real tol, NPRVertex* neighbor)
{
    List<NPRVertex*> result;
    Heap<real, NPRVertex*, greater<real> > Q;
    //List<NPRVertex*> Q;
    source->dist = 0.0;
    Q.push(source->dist, source);

    NPRVertex* target = NULL;
    real min_dist = bigval();
    while (!Q.empty()) {
        NPRVertex* cur_vertex = Q.top_value();
        real cur_dist = cur_vertex->dist;
        Q.pop();

        if (cur_vertex->isProp(VP_ANCHOR)
            && cur_vertex != source
            && cur_vertex != neighbor) {
                real dist = cur_vertex->dist;
                if (dist < min_dist) {
                    target = cur_vertex;
                    min_dist = dist;
                }
                continue;
        }

        NPREdge::circulator ci = cur_vertex->edge_circulator();
        do {
            NPRVertex* v = (*ci)->t();
            real dist = (cur_vertex->p() - v->p()).abs();
            if (cur_dist + dist < v->dist && cur_dist + dist < tol) {
                v->dist = cur_dist + dist;
                v->prev = cur_vertex;
                Q.push(v->dist, v);
            }
        } while (++ci != cur_vertex->edge_circulator() && ci != 0);
    }

    if (target == NULL) return result;
    NPRVertex* cur_v = target;
    while (cur_v->prev != NULL) {
        result.push_back(cur_v);
        cur_v = cur_v->prev;
    }
    if (cur_v == source) {
        //result.push_back(cur_v);
    } else {
        CONS("ERROR: source and target not connected!\r\n");
    }
    result.reverse();

    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

List<NPRVertex*> CNPRAlg::vertexDijkstraRun(NPRVertex *source, real radius)
{
    std::set<NPRVertex*> v_set;
    Heap<real, NPRVertex*, greater<real>> Q;
    //List<NPRVertex*> Q;
    source->dist = 0.0;
    Q.push(source->dist, source);

    NPRVertex* target = NULL;
    real min_dist = bigval();
    while (!Q.empty()) {
        NPRVertex* cur_vertex = Q.top_value();
        real cur_dist = cur_vertex->dist;
        Q.pop();

        bool onFeatureLine = false;
        NPREdge::circulator ci = cur_vertex->edge_circulator();
        do {
            if ((*ci)->count > 0 || ((*ci)->twin() && (*ci)->twin()->count > 0))
                onFeatureLine = true;
            NPRVertex* v = (*ci)->t();
            real dist = (cur_vertex->p() - v->p()).abs();
            if (cur_dist + dist < v->dist && cur_dist + dist < radius) {
                v->dist = cur_dist + dist;
                v->prev = cur_vertex;
                Q.push(v->dist, v);
            }
        } while (++ci != cur_vertex->edge_circulator() && ci != 0);

        if (!onFeatureLine)
            v_set.insert(cur_vertex);
    }

    List<NPRVertex*> result;
    foreach (vi, v_set, std::set<NPRVertex*>) {
        NPRVertex* v = *vi;
        result.push_back(v);
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

List<NPRFace*> CNPRAlg::faceDijkstraRun_old(NPRVertex* source, NPRVertex* target)
{
    List<NPRFace*> result;
    List<NPRFace*> Q;

    NPREdge::circulator ci = source->edge_circulator();
    do {
        NPRFace* f = (*ci)->face();
        f->m_ring = 1;
        Q.push_back(f);
    } while (++ci != source->edge_circulator() && ci != 0);

    NPRFace* cur_f = NULL;
    while (!Q.empty()) {
        NPRFace* cur_face = Q.front();
        int cur_ring = cur_face->m_ring;
        Q.pop_front();

        NPRFace::Edges e_list = cur_face->edges();

        bool done = false;
        foreach (ei, e_list, NPRFace::Edges) {
            NPRVertex* v = (*ei)->s();
            if (v == target) {
                cur_f = cur_face;
                done = true;
            }
        }
        if (done) break;

        foreach (ei, e_list, NPRFace::Edges) {
            NPREdge* e = *ei;
            if (e->twin()) {
                NPRFace* f = e->twin()->face();
                if (f->m_ring == 0) {
                    f->m_ring = 1 + cur_ring;
                    f->m_prev = cur_face;
                    Q.push_back(f);
                }
            }
        }
    }

    if (cur_f == NULL) {
        CONS("ERROR: source and target belong to two separate component.\r\n");
        ASSERT(false);
    } else {
        while (cur_f != NULL) {
            result.push_back(cur_f);
            cur_f = cur_f->m_prev;
        }
    }

    result.reverse();

    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
// The resulting shortest path does not cross any red edges (count > 0).
// *** distance measured in approx geodesic dist ***
// *** m_ring is used as a seed so we don't need to reinitialize every time ***
/////////////////////////////////////////////////////////////////////////////////////////
List<NPRFace*> CNPRAlg::faceDijkstraRun(NPRMesh& mesh, NPRVertex* source, NPRVertex* target, real radius)
{
    unsigned int seed = m_seedCount;
    m_seedCount++;
    if (m_seedCount == 0) { dijkstraReset(&mesh); m_seedCount++; }

    List<NPRFace*> result;
    //List<NPRFace*> Q;
    Heap<real, NPRFace*, greater<real> > Q;

    NPREdge::circulator ci = source->edge_circulator();
    do {
        NPRFace* f = (*ci)->face();
        f->m_ring = seed;
        f->m_dist = (f->centroid()-source->p()).abs();
        f->m_prev = NULL;
        //Q.push(f->m_ring, f);
        Q.push((f->centroid()-source->p()).abs(), f);
    } while (++ci != source->edge_circulator() && ci != 0);

    NPRFace* cur_f = NULL;
    while (!Q.empty()) {
        NPRFace* cur_face = Q.top_value();
        real dist_from_s = cur_face->m_dist;
        //CONS("%f\r\n", dist_from_s);
        int cur_ring = cur_face->m_ring;
        Q.pop();

        NPRFace::Edges e_list = cur_face->edges();

        bool done = false;
        foreach (ei, e_list, NPRFace::Edges) {
            NPRVertex* v = (*ei)->s();
            if (v == target) {
                cur_f = cur_face;
                done = true;
            }
        }
        if (done) break;

        foreach (ei, e_list, NPRFace::Edges) {
            NPREdge* e = *ei;
            if (e->twin()) {
                if (e->count > 0 || e->twin()->count > 0) continue;
                NPRFace* f = e->twin()->face();
                //if (f->v0()->isProp(VP_ANCHOR) && f->v0() != target) continue;
                //if (f->v1()->isProp(VP_ANCHOR) && f->v1() != target) continue;
                //if (f->v2()->isProp(VP_ANCHOR) && f->v2() != target) continue;
                if (f->m_ring != seed) {
                    real dist = dist_from_s + (f->centroid() - cur_face->centroid()).abs();
                    if (dist < radius) {
                        f->m_ring = seed;
                        f->m_prev = cur_face;
                        f->m_dist = dist;
                        Q.push(dist, f);
                        //Q.push(f->m_ring, f);
                    }
                } else {
                    real dist = dist_from_s + (f->centroid() - cur_face->centroid()).abs();
                    if (dist < f->m_dist && dist < radius) {
                        f->m_dist = dist;
                        f->m_prev = cur_face;
                        Q.push(dist, f);
                    }
                }
            }
        }
    }

    if (cur_f == NULL) {
        return result; // return an empty set
        //CONS("ERROR: source and target belong to two separate component.\r\n");
        //ASSERT(false);
    } else {
        while (cur_f != NULL) {
            result.push_back(cur_f);
            cur_f = cur_f->m_prev;
        }
    }

    result.reverse();

    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

List<NPRFace*> CNPRAlg::faceDijkstraRun(NPRFace* source, NPRFace* target)
{
    List<NPRFace*> result;
    List<NPRFace*> Q;

    source->m_ring = 1;
    Q.push_back(source);

    NPRFace* cur_f = NULL;
    while (!Q.empty()) {
        NPRFace* cur_face = Q.front();
        int cur_ring = cur_face->m_ring;
        Q.pop_front();

        NPRFace::Edges e_list = cur_face->edges();

        if (cur_face == target) {
            cur_f = cur_face;
            break;
        }

        foreach (ei, e_list, NPRFace::Edges) {
            NPREdge* e = *ei;
            if (e->twin()) {
                NPRFace* f = e->twin()->face();
                if (f->m_ring == 0) {
                    f->m_ring = 1 + cur_ring;
                    f->m_prev = cur_face;
                    Q.push_back(f);
                }
            }
        }
    }

    if (cur_f == NULL) {
        CONS("ERROR: source and target belong to two separate component.\r\n");
        ASSERT(false);
    } else {
        while (cur_f != NULL) {
            result.push_back(cur_f);
            cur_f = cur_f->m_prev;
        }
    }

    result.reverse();

    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

int CNPRAlg::partitionFaces(NPRVertex* source, real radius)
{
    int partitions = 0;
    Heap<real, NPRFace*, greater<real> > Q;

    NPREdge::circulator ci = source->edge_circulator();
    NPREdge::circulator begin = ci;
    do {
        NPREdge* e = *ci;
        if (e->count > 0 || (e->twin() && e->twin()->count > 0)) {
            begin = ci;
            break;
        }
    } while (++ci != source->edge_circulator() && ci != 0);
    ci = begin;
    do {
        if ((*ci)->count > 0 ||
            ((*ci)->twin() && (*ci)->twin()->count > 0)) partitions++;
        NPRFace* f = (*ci)->face();
        f->m_ring = partitions;
        Q.push((f->centroid()-source->p()).abs(), f);
    } while (++ci != begin && ci != 0);

    if (partitions == 0) return 0;

    while (!Q.empty()) {
        NPRFace* cur_face = Q.top_value();
        real dist_from_s = Q.top_key();
        int cur_ring = cur_face->m_ring;
        Q.pop();

        NPRFace::Edges e_list = cur_face->edges();

        foreach (ei, e_list, NPRFace::Edges) {
            NPREdge* e = *ei;
            if (e->twin()) {
                if (e->count > 0 || e->twin()->count > 0) continue;
                //if (e->dihedralAngle() > m_ridgeAngle) continue;
                NPRFace* f = e->twin()->face();
                if (f->m_ring == 0) {
                    f->m_ring = cur_ring;
                    f->m_prev = cur_face;
                    dist_from_s += (f->centroid() - cur_face->centroid()).abs();
                    if (dist_from_s > radius) continue;
                    Q.push(dist_from_s, f);
                }
            }
        }
    }
    return partitions;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::vHullToMesh()
{
    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        SVertex* prev = &stroke->front();
        foreach (svi, (*stroke), Stroke) {
            SVertex* vi = &*svi;
            if (vi == prev) continue;
            NPREdge* e = prev->mapped_vertex->getEdgeTo(vi->mapped_vertex);
            if (e) {
                e->count = m_drawerB.getMaxTol();
                if (e->twin()) e->twin()->count = m_drawerB.getMaxTol();
            }

            prev = vi;
        }

        if (stroke->circular) {
            NPREdge* e =
                stroke->back().mapped_vertex->getEdgeTo(
                stroke->front().mapped_vertex);
            if (e) {
                e->count = m_drawerB.getMaxTol();
                if (e->twin()) e->twin()->count = m_drawerB.getMaxTol();
            }
        }
    }


    /*
    generateNormals(m_meshA, 0, 1);// use coords in 0 and store normals in 1

    updateVoxelGrid();
    m_drawerB.setVoxelGrid(&m_vgrid);

    m_strokes.clear();

    m_meshA.clearSensor().modify();
    */
}

/////////////////////////////////////////////////////////////////////////////////////////
// Copy mesh1 to mesh2
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::copyMesh(NPRMesh* mesh1, NPRMesh* mesh2)
{
    mesh2->clear();
    *mesh2 = *mesh1;
    //m_path = "./temp.dat";
    //outputCoreMeshV2(*mesh1);
    //loadData(m_path, *mesh2);

    /*
    Array<NPRVertex*> v_list(mesh1->nVertices());
    int i = 0;
    foreach(vi, mesh1->vertices(), NPRMesh::Vertices) {
        NPRVertex* v = mesh2->addVertex(vi->p());
        v_list[i] = v;
        i++;
    }

    foreach(fi, mesh1->faces(), NPRMesh::Faces) {
        List<NPRVertex*> vs;
        foreach (ei, fi->edges(), List<NPREdge*>) {
            NPRVertex* v = (*ei)->s();
            vs.push_back(v_list[v->m_index]);
        }
        mesh2->addFace(vs);
    }
    */
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPRMesh& CNPRAlg::getActiveMesh()
{
    switch (m_activeMesh) {
        case meshA:
            return m_meshA;
        case meshB:
            return m_meshA;
        case visualhull:
            return m_vHull;
        default:
            return m_meshA;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

real CNPRAlg::computeAveMinDist(NPRMesh& mesh) const
{
    Array<real> minDist(mesh.nVertices(), -1.0);

    foreach_const (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRMesh::Vertices::const_iterator itr = vi;
        itr++;
        for (; itr != mesh.vertices().end(); itr++) {
            real dist = (vi->p() - itr->p()).abs();
            if (minDist[vi->m_index] == -1 ||
                dist < minDist[vi->m_index])
                minDist[vi->m_index] = dist;
            if (minDist[itr->m_index] == -1 ||
                dist < minDist[itr->m_index])
                minDist[itr->m_index] = dist;
        }
    }

    real result = 0;
    foreach_const (i, minDist, Array<real>) {
        result += *i;
    }

    return result/mesh.nVertices();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::assignVertexIndex(NPRMesh& mesh)
{
    int count = 0;
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        vi->m_index = count;
        count++;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

/*
void CNPRAlg::splitEdge(NPRMesh& mesh, NPREdge* edge, real ratio)
{
    ASSERT(ratio > -epsilon() && ratio < 1.0 + epsilon());
    NPRFace* f1 = edge->face();
    ASSERT(f1->triangular());
    NPRFace* f2 = NULL;
    if (edge->twin()) {
        f2 = edge->twin()->face();
        ASSERT(f2->triangular());
    }

    // add new vertex
    Point3D p = edge->s()->p()*ratio + edge->t()->p()*(1-ratio);
    NPRVertex* new_v = mesh.addVertex(p);

    // remove f1
    int degree = f1->edges().size();
    List<NPRVertex*> v_list1;
    List<NPRVertex*> v_list2;
    v_list1.push_back(new_v);
    NPREdge* cur_edge = edge;
    for (int i=0; i<=degree/2; i++) {
        v_list1.push_back(cur_edge->t());
        cur_edge = cur_edge->nextInFace();
    }
    v_list2.push_back(v_list1.back());
    while (cur_edge != edge) {
        v_list2.push_back(cur_edge->t());
        cur_edge = cur_edge->nextInFace();
    }
    v_list2.push_back(new_v);
    mesh.removeFace(f1);
    mesh.addFace(v_list1);
    mesh.addFace(v_list2);

    // remove f2
    if (f2 == NULL) return;
    degree = f2->edges().size();
    v_list1.clear();
    v_list2.clear();
    v_list1.push_back(new_v);
    cur_edge = edge->twin();
    for (int i=0; i<=degree/2; i++) {
        v_list1.push_back(cur_edge->t());
        cur_edge = cur_edge->nextInFace();
    }
    v_list2.push_back(v_list1.back());
    while (cur_edge != edge->twin()) {
        v_list2.push_back(cur_edge->t());
        cur_edge = cur_edge->nextInFace();
    }
    v_list2.push_back(new_v);
    mesh.removeFace(f2);
    mesh.addFace(v_list1);
    mesh.addFace(v_list2);
}
*/

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPRVertex* CNPRAlg::splitEdge(NPRMesh& mesh, NPREdge* edge, Point3D& p, bool approximate)
{
    ASSERT(edge->twin());
    if (edge->length() < epsilon()) {
        CONS("Error: edge of length 0");
        ASSERT(false);
    }
    int e_count = edge->count;
    int twin_count = edge->twin()->count;
    real vis_ratio1 = edge->face()->m_visRatio;
    real vis_ratio2 = edge->twin()->face()->m_visRatio;
    NPRVertex* s = edge->s();
    NPRVertex* t = edge->t();

    Point3D p1 = edge->s()->p();
    Point3D p2 = edge->t()->p();
    Point3D v = (p2-p1).unit();
    Point3D cut = p1 + v*(v*(p-p1));
    if (approximate) {
        if ((cut-p1).abs() < m_tol && !edge->s()->isProp(VP_ANCHOR)) return edge->s();
        if ((cut-p2).abs() < m_tol && !edge->t()->isProp(VP_ANCHOR)) return edge->t();
    }
    NPRVertex* result = mesh.addVertex(cut);
    //edge->count = 1;
    mesh.refineEdge(edge, result);

    // copy visRatio
    NPREdge* e1 = s->getEdgeTo(result);
    NPREdge* e2 = result->getEdgeTo(t);
    ASSERT(e1 != NULL);
    ASSERT(e2 != NULL);
    e1->face()->m_visRatio = vis_ratio1;
    e2->face()->m_visRatio = vis_ratio1;
    e1->twin()->face()->m_visRatio = vis_ratio2;
    e2->twin()->face()->m_visRatio = vis_ratio2;

    // copy edge count
    e1->count = e_count;
    e1->twin()->count = twin_count;
    e2->count = e_count;
    e2->twin()->count = twin_count;
    e2->nextInFace()->count = e2->nextInFace()->twin()->count;
    e1->prevInFace()->count = e1->prevInFace()->twin()->count;
    e1->twin()->nextInFace()->count = e1->twin()->nextInFace()->twin()->count;
    e2->twin()->prevInFace()->count = e2->twin()->prevInFace()->twin()->count;
   return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::computeEdges(NPRMesh& mesh, real radius)
{
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = &*vi;
        if (!v->marked) {
            v->marked = true;
            NPRVertex* next = NULL;
            m_strokes.push_back();
            Stroke* stroke = &m_strokes.back();
            while (v != NULL) {
                SVertex sv(v);
                stroke->push_back(sv);
                next = removeClosePoints(mesh, v, radius);
                if (next == NULL) break;
                v->next = next;
                v = next;
            }
            if (stroke->size() < 5) {
                m_strokes.pop_back();
            }
        }
    }
    m_drawerB.setStrokes(&m_strokes);
    CONS("Generated %d strokes.\r\n", m_strokes.size());
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPRVertex* CNPRAlg::removeClosePoints(NPRMesh& mesh, NPRVertex* v, real radius)
{
     NPRVertex* next = NULL;
     real maxDistance = 0.0;

     foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        real distance = (vi->p() - v->p()).abs();
        if( distance < radius && !vi->marked ){
            vi->marked = true;
            if (abs(maxDistance - distance) < epsilon()) {
                if (vi->p().abs() < next->p().abs()) {
                    maxDistance = distance;
                    next = &*vi;
                }
            } else if( maxDistance < distance) {
                maxDistance = distance;
                next = &*vi;
            }
        }
     }
     return next;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

real CNPRAlg::geodesicDist(NPRMesh& mesh, NPRVertex* source, NPRVertex* target)
{
    real dist = 0.0;
    dijkstraReset(&mesh);
    List<NPRVertex*> path = vertexDijkstraRun(mesh, source, target);
    NPRVertex* prev = source;
    foreach (vi, path, List<NPRVertex*>) {
        dist += ((*vi)->p() - prev->p()).abs();
    }
    return dist;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::computeAllPairGeodesicDist(NPRMesh& mesh)
{
    /*
    m_distMap.clear();
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        //dijkstraReset(&mesh);
        //vertexDijkstraRun(mesh, &*vi, NULL);
        vertexBellmanFord(mesh, &*vi, NULL);
        foreach (oi, mesh.vertices(), NPRMesh::Vertices) {
            m_distMap[std::pair<NPRVertex*, NPRVertex*>(&*vi, &*oi)] = oi->dist;
        }
    }
    */
    int num_v = mesh.nVertices();
    assignVertexIndex(mesh);
    real* distMatrix = new real[num_v*num_v];
    for (int i=0; i<num_v; i++) {
        for (int j=0; j<num_v; j++) {
            if (i == j) {
                distMatrix[i*num_v+i] = 0;
            } else {
                distMatrix[i*num_v+j] = bigval()/2.0; // no overflow
            }
        }
    }
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        int row = ei->s()->m_index;
        int col = ei->t()->m_index;
        real dist = (ei->s()->p() - ei->t()->p()).abs();
        distMatrix[row*num_v+col] = dist;
        distMatrix[col*num_v+row] = dist;
    }

    for (int k=0; k<30; k++) {
        for (int i=0; i<num_v; i++) {
            for (int j=0; j<num_v; j++) {
                real dist = distMatrix[i*num_v+k] + distMatrix[k*num_v+j];
                if (dist < distMatrix[i*num_v+j]) {
                    distMatrix[i*num_v+j] = dist;
                }
            }
        }
    }

    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        foreach (oi, mesh.vertices(), NPRMesh::Vertices) {
            int row = vi->m_index;
            int col = oi->m_index;
            if (distMatrix[row*num_v+col] < bigval()/2.0+1.0) {
                m_distMap[std::pair<NPRVertex*, NPRVertex*>(&*vi, &*oi)] = distMatrix[row*num_v+col];
            }
        }
    }

    delete [] distMatrix;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Bellman-Ford algorithm with run time O(|V|*|E|)
/////////////////////////////////////////////////////////////////////////////////////////

List<NPRVertex*> CNPRAlg::vertexBellmanFord(NPRMesh& mesh, NPRVertex* source, NPRVertex* target)
{
    List<NPRVertex*> result;

    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        vi->dist = bigval()/2.0;
        vi->prev = NULL;
    }
    source->dist = 0.0;

    int num_v = mesh.nVertices();

    for (int i=0; i<num_v; i++) {
        foreach (ei, mesh.edges(), NPRMesh::Edges) {
            real len = ei->length();
            if (ei->t()->dist > ei->s()->dist + len) {
                ei->t()->dist = ei->s()->dist + len;
                ei->t()->prev = ei->s();
            }
        }
    }

    if (target == NULL) return result;
    // tracing
    NPRVertex* cur = target;
    while (cur != NULL) {
        result.push_back(cur);
        cur = cur->prev;
    }
    result.reverse();
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::resizeToUnitBox(NPRMesh& mesh)
{
    double factor = epsilon();
	foreach(vi, mesh.vertices(), Mesh3D::Vertices) {
        if (abs(vi->p().x()) > factor) { factor = abs(vi->p().x()); }
        if (abs(vi->p().y()) > factor) { factor = abs(vi->p().y()); }
        if (abs(vi->p().z()) > factor) { factor = abs(vi->p().z()); }
	}
    scale(mesh, 1.0/factor);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::flipZ(NPRMesh& mesh)
{
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        vi->p().z() *= -1;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Fast only if target has only a few faces (works for man-made models)
/////////////////////////////////////////////////////////////////////////////////////////

real CNPRAlg::computeMeshMapping(NPRMesh& source, NPRMesh& target)
{
    foreach (fi, target.faces(), NPRMesh::Faces) {
        fi->computePlaneEq();
    }
    real ave_dist = 0;
    foreach (vi, source.vertices(), NPRMesh::Vertices) {
        Point3D p = vi->p();
        real min_dist = bigval();
        foreach (fi, target.faces(), NPRMesh::Faces) {
            Point3D p_in_f = fi->closestPoint(p);
            real dist = (p-p_in_f).abs();
            if (dist < min_dist) {
                vi->nearestPoint = p_in_f;
                vi->mapped_face = &*fi;
                min_dist = dist;
            }
        }
        ave_dist += min_dist;
    }
    ave_dist /= source.nVertices();
    return ave_dist;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Fast mesh to mesh mapping using voxel grid
/////////////////////////////////////////////////////////////////////////////////////////

real CNPRAlg::computeFastMeshMapping(NPRMesh& source, NPRMesh& target)
{
    foreach (fi, target.faces(), NPRMesh::Faces) {
        fi->computePlaneEq();
    }
    real ave_dist = 0;
    int count = 0;
    foreach (vi, source.vertices(), NPRMesh::Vertices) {
        /*
        Point3D p = vi->p();
        real min_dist = bigval();
        Point3D vox_index = m_vgrid.inverseMap(p);
        for (int i=int(vox_index.x())-1; i<=int(vox_index.x())+1; i++) {
            if (i < 0 || i >= m_vgrid.getWidth()) continue;
            for (int j=int(vox_index.y()-1); j<=int(vox_index.y())+1; j++) {
                if (j < 0 || j >= m_vgrid.getHeight()) continue;
                for (int k=int(vox_index.z())-1; k<=int(vox_index.z())+1; k++) {
                    if (k < 0 || k >= m_vgrid.getDepth()) continue;

                    Voxel* vox = m_vgrid.vox(i,j,k);
                    if (vox->f_list.empty()) continue;
                    for (std::set<NPRFace*>::iterator fi = vox->f_list.begin();
                        fi != vox->f_list.end(); fi++) {
                            Point3D p_in_f = (*fi)->closestPoint(p);
                            real dist = (p-p_in_f).abs();
                            if (dist < min_dist) {
                                vi->nearestPoint = p_in_f;
                                min_dist = dist;
                            }
                    }
                }
            }
        }
        if (min_dist > 1.0) continue;
        */
        NPRFace* min_face = getNearestPoint(&*vi, target);
        if (min_face == NULL) continue;
        vi->mapped_face = min_face;
        real min_dist = (vi->p() - vi->nearestPoint).abs();
        ave_dist += min_dist;
        count++;
    }
    ave_dist /= count;
    return ave_dist;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Fast mesh to mesh mapping using voxel grid
/////////////////////////////////////////////////////////////////////////////////////////

real CNPRAlg::computeFastMeshMapping(Strokes& source, NPRMesh& target)
{
    foreach (fi, target.faces(), NPRMesh::Faces) {
        fi->computePlaneEq();
    }
    real ave_dist = 0;
    int count = 0;
    foreach (si, source, Strokes) {
        Stroke& stroke = *si;
        foreach (svi, stroke, Stroke) {
            SVertex* sv = &*svi;
            NPRFace* min_face = getNearestPoint(sv->vertex, target);
            sv->mapped_face = NULL;
            sv->mapped_point = Point3D(0.0, 0.0, 0.0);
            if (min_face == NULL) continue;
            sv->mapped_face = min_face;
            sv->vertex->mapped_face = min_face;
            sv->mapped_point = sv->vertex->nearestPoint;
            real min_dist = (sv->vertex->p() - sv->vertex->nearestPoint).abs();
            ave_dist += min_dist;
            count++;
        }
    }
    ave_dist /= count;
    return ave_dist;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPRFace* CNPRAlg::getNearestPoint(NPRVertex* v, NPRMesh& target)
{
    Point3D p = v->p();
    Point3D nearestPoint;
    NPRFace* result = getNearestPoint(p, target, nearestPoint);
    v->nearestPoint = nearestPoint;
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPRFace* CNPRAlg::getNearestPoint(Point3D& p, NPRMesh& target, Point3D& nearestPoint, NPRVertex* exception)
{
    real min_dist = bigval();
    NPRFace* min_face = NULL;
    Point3D vox_index = m_vgrid.inverseMap(p);
    for (int i=int(vox_index.x())-1; i<=int(vox_index.x())+1; i++) {
        if (i < 0 || i >= m_vgrid.getWidth()) continue;
        for (int j=int(vox_index.y()-1); j<=int(vox_index.y())+1; j++) {
            if (j < 0 || j >= m_vgrid.getHeight()) continue;
            for (int k=int(vox_index.z())-1; k<=int(vox_index.z())+1; k++) {
                if (k < 0 || k >= m_vgrid.getDepth()) continue;

                Voxel* vox = m_vgrid.vox(i,j,k);
                if (vox->f_list.empty()) continue;
                for (std::set<NPRFace*>::iterator fi = vox->f_list.begin();
                    fi != vox->f_list.end(); fi++) {
                        Point3D p_in_f = (*fi)->closestPoint(p);
                        real dist = (p-p_in_f).abs();
                        if (dist < min_dist - m_tol) {
                            nearestPoint = p_in_f;
                            min_dist = dist;
                            min_face = *fi;
                        } else if (min_face != NULL && dist < min_dist + m_tol) {
                            real min_vis_ratio = min_face->m_visRatio;
                            real vis_ratio = (*fi)->m_visRatio;
                            if (vis_ratio < min_vis_ratio ||
                                (abs(vis_ratio - min_vis_ratio) < epsilon() &&
                                dist < min_dist)) {
                                    nearestPoint = p_in_f;
                                    min_dist = dist;
                                    min_face = *fi;
                            }
                        }
                }
            }
        }
    }
    return min_face;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPRFace* CNPRAlg::getNearestPoint_slow(NPRVertex *v, NPRMesh &target)
{
    Point3D p = v->p();
    NPRFace* min_face = NULL;
    real min_dist = bigval();
    foreach (fi, target.faces(), NPRMesh::Faces) {
        fi->computePlaneEq();
        Point3D p_in_f = fi->closestPoint(p);
        real dist = (p-p_in_f).abs();
        if (dist < min_dist - m_mapThreshold) {
            v->nearestPoint = p_in_f;
            min_dist = dist;
            min_face = &*fi;
        } else if (min_face != NULL && dist < min_dist + m_mapThreshold) {
            real min_vis_ratio = min_face->m_visRatio;
            real vis_ratio = fi->m_visRatio;
            if (vis_ratio > min_vis_ratio + m_visThreshold ||
                (vis_ratio > min_vis_ratio - m_visThreshold &&
                dist < min_dist)) {
                    v->nearestPoint = p_in_f;
                    min_dist = dist;
                    min_face = &*fi;
            }
        }
    }
    return min_face;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

List<NPRFace*> CNPRAlg::getNearestPoints_slow(NPRVertex *v, NPRMesh &target)
{
    List<NPRFace*> result;
    Point3D p = v->p();
    NPRFace* min_face = NULL;
    real min_dist = bigval();
    foreach (fi, target.faces(), NPRMesh::Faces) {
        if (fi->area() < epsilon()) continue;
        fi->computePlaneEq();
        Point3D p_in_f = fi->closestPoint(p);
        real dist = (p-p_in_f).abs();
        if (dist < min_dist) {
            v->nearestPoint = p_in_f;
            min_dist = dist;
            min_face = &*fi;
        }
    }

    foreach (fi, target.faces(), NPRMesh::Faces) {
        if (fi->area() < epsilon()) continue;
        fi->computePlaneEq();
        Point3D p_in_f = fi->closestPoint(p);
        real dist = (p-p_in_f).abs();
        if (dist < min_dist + m_mapThreshold) {
            result.push_back(&*fi);
        }
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPRFace* CNPRAlg::getNearestPoint(NPRVertex *v, CNPRAlg::FaceSet &faces)
{
    Point3D p = v->p();
    NPRFace* min_face = NULL;
    real min_dist = bigval();
    FaceSet::iterator i = faces.begin();
    for (i; i != faces.end(); i++) {
        NPRFace* f = *i;
        f->computePlaneEq();
        Point3D p_in_f = f->closestPoint(p);
        real dist = (p-p_in_f).abs();
        if (dist < min_dist) {
            v->nearestPoint = p_in_f;
            min_dist = dist;
            min_face = f;
        }
    }
    return min_face;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPREdge* CNPRAlg::getNearestPoint_crease(NPRVertex* vertex, NPRMesh& target)
{
    NPREdge* min_edge = NULL;
    real min_dist = bigval();
    Point3D p = vertex->p();
    foreach (ei, target.edges(), NPRMesh::Edges) {
        if (!ei->twin()) continue;
        if (ei->count > 0 || ei->twin()->count > 0) continue;
        real angle = ei->dihedralAngle();
        if (angle > m_ridgeAngle) {
            Point3D p1 = ei->s()->p();
            Point3D p2 = ei->t()->p();
            Point3D v = (p2-p1).unit();
            Point3D projection = v*(v*(p-p1));
            if (projection.abs() > (p2-p1).abs() || projection*v < 0) continue;
            real dist = (p-p1-projection).abs();
            if (dist < min_dist - m_mapThreshold && dist < 2*m_tol) {
                min_dist = dist;
                min_edge = &*ei;
                vertex->nearestPoint = p1+projection;
            } else if (min_edge != NULL
                && dist < min_dist + m_mapThreshold && dist < 2*m_tol) {
                    real min_vis_ratio = min_edge->face()->m_visRatio +
                        min_edge->twin()->face()->m_visRatio;
                    real vis_ratio = ei->face()->m_visRatio +
                        ei->twin()->face()->m_visRatio;
                    if (vis_ratio > min_vis_ratio + m_visThreshold ||
                        (vis_ratio > min_vis_ratio - m_visThreshold &&
                        dist < min_dist)) {
                            min_dist = dist;
                            min_edge = &*ei;
                            vertex->nearestPoint = p1+projection;
                    }
            }
        }
    }
    return min_edge;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

Point3D CNPRAlg::getNearestPoint(Point3D &from1, Point3D &from2, Point3D &to1, Point3D &to2, int samples) const
{
    Point3D result(from1);
    real min_dist = bigval();
    for (int i=0; i<=samples; i++) {
        real ratio = double(i)/double(samples);
        Point3D p = from1*(1-ratio) + from2*ratio;
        real dist = p2lDist(p, to1, to2);
        if (dist < min_dist) {
            result = p;
            min_dist = dist;
        }
    }
    return result;
    /*
    if (level == 0) {
        real dist1 = p2lDist(from1, to1, to2);
        real dist2 = p2lDist(from2, to1, to2);
        return dist1<dist2 ? from1:from2;
    }
    Point3D mid = (from1+from2)*0.5;
    Point3D p1 = getNearestPoint(from1, mid, to1, to2, level-1);
    Point3D p2 = getNearestPoint(mid, from2, to1, to2, level-1);
    real dist1 = p2lDist(p1, to1, to2);
    real dist2 = p2lDist(p2, to1, to2);
    return dist1<dist2 ? p1:p2;
    */
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

real CNPRAlg::p2lDist(Point3D &p, Point3D &l1, Point3D &l2) const
{
    if ((l1-l2).abs() < epsilon()) return (p-l1).abs();
    Point3D v = (l2-l1).unit();
    Point3D projection = v*(v*(p-l1));
    return ((p-l1)-projection).abs();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::cutMesh(NPRMesh& mesh, Stroke& stroke, bool debug)
{
    List<NPRVertex*> v_list;
    foreach (svi, stroke, Stroke) {
        SVertex* sv = &*svi;
        if (sv->vertex != NULL && sv->vertex->mapped != NULL) {
            sv->vertex->mapped->marked = true;
            if (debug) MB("T junction!");
            sv->vertex->mapped->marked = false;
            v_list.push_back(sv->vertex->mapped);
            continue;
        }
        NPRVertex* v = NULL;
        NPREdge* edge = getNearestPoint_crease(sv->vertex, mesh);
        if (edge != NULL) {
            //v = splitEdge(mesh, edge, sv->vertex->p());
            if (debug) CONS("crease\r\n");
            v = splitEdge(mesh, edge, sv->vertex->nearestPoint);
        } else {
            if (debug) CONS("non-crease\r\n");
            NPRFace* f = getNearestPoint_slow(sv->vertex, mesh);
            v = splitFace(mesh, f, sv->vertex->nearestPoint);
        }
        //fixBadTriangles(mesh, v);
        //v->addProp(VP_ANCHOR);
        if (v->secondPos.abs() < epsilon()
            || (v->p()-v->secondPos).abs() >= (v->p()-sv->vertex->p()).abs()) {
                v->secondPos = sv->vertex->p();
        }
        v->sv = sv;
        sv->vertex->mapped = v;
        sv->mapped_vertex = v;
        if (v->secondPos.abs() < epsilon()) {
            CONS("anchor maps to 0.\r\n");
        }
        v_list.push_back(v);
    }
    foreach (vi, v_list, List<NPRVertex*>) {
        (*vi)->addProp(VP_ANCHOR);
    }
    if (debug) MB("Linux rocks!");

    List<NPRVertex*> anchor_list;
    NPRVertex* prev = v_list.front();
    anchor_list.push_back(prev);
    foreach (vi, v_list, List<NPRVertex*>) {
        if (vi == v_list.begin()) continue;
        NPRVertex* cur = *vi;
        if (cur == prev) continue;
        NPREdge* e = prev->getEdgeTo(cur);
        if (e != 0) {
            anchor_list.push_back(cur);
            e->count = m_drawerB.getMaxTol();
            if (e->twin())
                e->twin()->count = m_drawerB.getMaxTol();
        } else {
            dijkstraReset(&mesh);
            Point3D p1 = prev->sv->vertex->p();
            Point3D p2 = cur->sv->vertex->p();
            List<NPRFace*> f_list = faceDijkstraRun(mesh, prev, cur);
            if (debug) {
                prev->marked = true;
                cur->marked = true;
                foreach (fi, f_list, List<NPRFace*>) {
                    (*fi)->color() = Color3d(1.0, 0.0, 0.0);
                }
                MB("1");
                prev->marked = false;
                cur->marked = false;
                foreach (fi, f_list, List<NPRFace*>) {
                    (*fi)->color() = Color3d(1.0, 1.0, 1.0);
                }
            }
            ///*
            cutThroughFaces(mesh, f_list, prev->p(), cur->p());
            dijkstraReset(&mesh);
            List<NPRVertex*> v_path = vertexDijkstraRun(mesh, prev, cur);
            if (v_path.front() == prev) v_path.pop_front();
            if (v_path.back() != cur) v_path.push_back(cur);
            real path_dist = pathDist(v_path);
            path_dist += (prev->p() - v_path.front()->p()).abs();
            // Mapped path is way too long, likely to be a wrong match.
            if (path_dist > 10*(p1-p2).abs() ||
                v_path.size() == 1) {
                prev->marked = true;
                cur->marked = true;
                //foreach (vpi, v_path, List<NPRVertex*>) {
                //    (*vpi)->marked = true;
                //}
                MB("forgive me. %f > 10*%f", path_dist, (p1-p2).abs());
                prev->marked = false;
                cur->marked = false;
                //foreach (vpi, v_path, List<NPRVertex*>) {
                //    (*vpi)->marked = false;
                //}
                prev = cur;
                continue;
            }
            int v_count = 1;
            int v_size = v_path.size();
            foreach (vpi, v_path, List<NPRVertex*>) {
                NPRVertex* cur2 = *vpi;
                //if (cur2 == prev) {
                //    v_count++;
                //    continue;
                //}
                real ratio = double(v_count)/double(v_size);
                Point3D mapped = p1*(1-ratio) + p2*ratio;
                cur2->secondPos = mapped;
                v_count++;
                anchor_list.push_back(cur2);
                NPREdge* edge = prev->getEdgeTo(cur2);
                if (edge) {
                    edge->count = m_drawerB.getMaxTol();
                    if (edge->twin())
                        edge->twin()->count = m_drawerB.getMaxTol();
                } else {
                    CONS("Path has %d vertices.\r\n", v_path.size());
                    CONS("Path broken %f.\r\n", path_dist);
                }
                cur2->addProp(VP_ANCHOR);
                prev = cur2;
            }
            if (debug) MB("2");
            //*/
        }
        prev = cur;
    }

    CONS("Number of Anchors: %d\r\n", anchor_list.size());
    /*
    prev = anchor_list.front();
    int count = 0;
    foreach (vi, anchor_list, List<NPRVertex*>) {
        //(*vi)->m_index = count;
        count++;
        if (vi == anchor_list.begin()) continue;
        NPRVertex* cur = *vi;
        NPREdge* e = prev->getEdgeTo(cur);
        if (e) {
            e->count = m_drawerB.getMaxTol();
            if (e->twin())
                e->twin()->count = m_drawerB.getMaxTol();
        } else {
            if (prev != cur)
                CONS("Path broken.\r\n");
            //prev->addProp(VP_ANCHOR);
            //cur->addProp(VP_ANCHOR);
        }
        prev = cur;
    }
    */
#if 0
    FaceSet f_activeSet;
    SVertex* prev = &stroke.front();
    foreach (svi, stroke, Stroke) {
        SVertex* sv = &*svi;
        if (svi == stroke.begin()) continue;
        NPRFace* prev_f = getNearestPoint_slow(prev->vertex, mesh);
        NPRFace* f = getNearestPoint_slow(sv->vertex, mesh);
        if (prev_f == f) {
            prev = sv;
            continue;
        }
        dijkstraReset(&mesh);
        List<NPRFace*> f_list = faceDijkstraRun(prev_f, f);
        cutThroughFaces(mesh, f_list);
        /*
        NPRFace* f = NULL;
        if (svi == stroke.begin()) {
            f = getNearestPoint_slow(sv->vertex, mesh);
        } else {
            f = getNearestPoint(sv->vertex, f_activeSet);
        }
        splitFace(mesh, f, sv->vertex->nearestPoint, f_activeSet);
        */
        prev = sv;
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::cutMesh2(NPRMesh& mesh, Stroke& stroke, bool debug)
{
    stroke.debug = debug;
    List<NPRVertex*> v_list;
    foreach (svi, stroke, Stroke) {
        //Array<NPRVertex*> active_list;
        std::set<NPRVertex*> active_list;
        SVertex* sv = &*svi;
        sv->m_states.clear();
        if (sv->vertex != NULL && sv->vertex->mapped != NULL) {
            sv->vertex->mapped->marked = true;
            if (debug) MB("T junction!");
            sv->vertex->mapped->marked = false;
            sv->vertex->mapped->remProp(VP_ANCHOR);
            active_list.insert(sv->vertex->mapped);
            sv->calcStates(active_list);
            continue;
        }
        if (stroke.circular && svi == (--stroke.end())) {
            foreach (sti, stroke.front().m_states, SVertex::States) {
                active_list.insert(sti->tv);
            }
            sv->calcStates(active_list);
            continue;
        }
        NPRVertex* v = NULL;
        NPREdge* edge = getNearestPoint_crease(sv->vertex, mesh);
        if (edge != NULL) {
            //v = splitEdge(mesh, edge, sv->vertex->p());
            if (debug) CONS("crease\r\n");
            v = splitEdge(mesh, edge, sv->vertex->nearestPoint);
            if (isnan(v->p().x()) || isnan(v->p().y()) || isnan(v->p().z())) {
                CONS("bad points by spliting crease.\r\n");
            }
            active_list.insert(v);
        } 
        else {
            if (debug) CONS("non-crease\r\n");
            List<NPRFace*> f_list = getNearestPoints_slow(sv->vertex, mesh);
            foreach (fi, f_list, List<NPRFace*>) {
                NPRFace* f = *fi;
                if (!f) continue;
                bool valid = true;
                foreach (ei, f->edges(), List<NPREdge*>) {
                    NPREdge* e = *ei;
                    if (!e) { valid = false; break; }
                    if (e->face() != f) { valid = false; break; }
                }
                if (!valid) continue;
                if (f->area() < epsilon()) continue;

                f->computePlaneEq();
                Point3D p_in_f = f->closestPoint(sv->vertex->p());
                if (isnan(p_in_f.x()) || isnan(p_in_f.y()) || isnan(p_in_f.z())) {
                    f->color() = Color3d(1.0, 0.0, 0.0);
                    f->v0()->marked = true;
                    f->v1()->marked = true;
                    f->v2()->marked = true;
                    CONS("area: %f\r\n", f->area());
                    CONS(" P: %f %f %f\r\n", f->v0()->p().x(), f->v0()->p().y(), f->v0()->p().z());
                    CONS(" P: %f %f %f\r\n", f->v1()->p().x(), f->v1()->p().y(), f->v1()->p().z());
                    CONS(" P: %f %f %f\r\n", f->v2()->p().x(), f->v2()->p().y(), f->v2()->p().z());
                    MB("Let her go because you love her.");
                }
                v = splitFace(mesh, f, p_in_f);
                NPREdge::circulator dummy;
                int degree = featureDegree(v, dummy);
                if (degree == 0) {
                    active_list.insert(v);
                } else if (debug) {
                    CONS("Candidate on other feature lines, ignored.\r\n");
                }
            }
        }
        /*
        if (debug) {
            foreach (vi, active_list, Array<NPRVertex*>) {
                (*vi)->marked = true;
            }
            MB("Check possible states");
            foreach (vi, active_list, Array<NPRVertex*>) {
                (*vi)->marked = false;
            }
        }
        */
        /*
        if (stroke.circular && (svi == stroke.begin() || svi == (--stroke.end()))) {
            foreach (vi, active_list, Array<NPRVertex*>) {
                (*vi)->marked = true;
            }
            MB("Don't give up!");
            foreach (vi, active_list, Array<NPRVertex*>) {
                (*vi)->marked = false;
            }
        }
        */
        sv->calcStates(active_list);
        //if (stroke.circular && (svi == stroke.begin() || svi == (--stroke.end()))) {
        //    CONS("SVertex has %d states\r\n", sv->m_states.size());
        //}
    }
    viterbi(mesh, &stroke);
    foreach (svi, stroke, Stroke) {
        SVertex* sv = &*svi;
        NPRVertex* v = sv->mapped_vertex;

        if (v->secondPos.abs() < epsilon()
            || (v->p()-v->secondPos).abs() >= (v->p()-sv->vertex->p()).abs()) {
                v->secondPos = sv->vertex->p();
        }
        v->sv = sv;
        sv->vertex->mapped = v;
        sv->mapped_vertex = v;
        if (v->secondPos.abs() < epsilon()) {
            CONS("anchor maps to 0.\r\n");
        }
        v_list.push_back(v);
    }
    foreach (vi, v_list, List<NPRVertex*>) {
        (*vi)->addProp(VP_ANCHOR);
    }
    if (debug) MB("Linux rocks!");
    if (!isValidMesh(mesh)) {
        CONS("mesh invalid...\r\n");
        MB("pause...");
    }

    List<NPRVertex*> anchor_list;
    NPRVertex* prev = v_list.front();
    anchor_list.push_back(prev);
    foreach (vi, v_list, List<NPRVertex*>) {
        if (vi == v_list.begin()) continue;
        NPRVertex* cur = *vi;
        if (cur == prev) continue;
        NPREdge* e = prev->getEdgeTo(cur);
        if (e != 0) {
            anchor_list.push_back(cur);
            e->count = m_drawerB.getMaxTol();
            if (e->twin())
                e->twin()->count = m_drawerB.getMaxTol();
        } else {
            //dijkstraReset(&mesh);
            Point3D p1 = prev->sv->vertex->p();
            Point3D p2 = cur->sv->vertex->p();
            List<NPRFace*> f_list = faceDijkstraRun(mesh, prev, cur);
            if (debug) {
                prev->marked = true;
                cur->marked = true;
                foreach (fi, f_list, List<NPRFace*>) {
                    (*fi)->color() = Color3d(1.0, 0.0, 0.0);
                }
                MB("1");
                prev->marked = false;
                cur->marked = false;
                foreach (fi, f_list, List<NPRFace*>) {
                    (*fi)->color() = Color3d(1.0, 1.0, 1.0);
                }
            }
            ///*
            if (!isValidMesh(mesh)) {
                CONS("before cut: mesh invalid...\r\n");
                MB("pause...");
            }
            cutThroughFaces(mesh, f_list, prev->p(), cur->p());
            if (!isValidMesh(mesh)) {
                CONS("after cut: mesh invalid...\r\n");
                MB("pause...");
            }
            //dijkstraReset(&mesh);
            List<NPRVertex*> v_path = vertexDijkstraRun(mesh, prev, cur);
            if (v_path.front() == prev) v_path.pop_front();
            if (v_path.back() != cur) v_path.push_back(cur);
            real path_dist = pathDist(v_path);
            path_dist += (prev->p() - v_path.front()->p()).abs();
            // Mapped path is way too long, likely to be a wrong match.
            if (path_dist > 10*(p1-p2).abs() || v_path.size() == 1) {
                prev->marked = true;
                cur->marked = true;
                //NPRVertex* prev_v;
                //foreach (vpi, v_path, List<NPRVertex*>) {
                //    (*vpi)->marked = true;
                    /*
                    if (vpi == v_path.begin()) {
                        prev_v = *vpi;
                        continue;
                    }
                    NPREdge* edge_debug = prev_v->getEdgeTo((*vpi));
                    if (edge_debug != NULL) {
                        edge_debug->count = 1;
                        CONS("Edge Len = %f\r\n", edge_debug->length());
                        MB("stupid...");
                    }
                    prev_v = (*vpi);
                    */
                //}
                if (debug) MB("forgive me. %f > 10*%f", path_dist, (p1-p2).abs());
                prev->marked = false;
                cur->marked = false;
                //foreach (vpi, v_path, List<NPRVertex*>) {
                //    (*vpi)->marked = false;
                //}
                prev = cur;
                continue;
            }
            int v_count = 1;
            int v_size = v_path.size();
            real dist = 0.0;
            foreach (vpi, v_path, List<NPRVertex*>) {
                NPRVertex* cur2 = *vpi;
                //if (cur2 == prev) {
                //    v_count++;
                //    continue;
                //}
                dist += (cur2->p() - prev->p()).abs();
                //real ratio = double(v_count)/double(v_size);
                real ratio = dist/path_dist;
                Point3D mapped = p1*(1-ratio) + p2*ratio;
                cur2->secondPos = mapped;
                v_count++;
                anchor_list.push_back(cur2);
                NPREdge* edge = prev->getEdgeTo(cur2);
                if (edge) {
                    edge->count = m_drawerB.getMaxTol();
                    if (edge->twin())
                        edge->twin()->count = m_drawerB.getMaxTol();
                } else {
                    CONS("Path has %d vertices.\r\n", v_path.size());
                    CONS("Path broken %f.\r\n", path_dist);
                }
                cur2->addProp(VP_ANCHOR);
                prev = cur2;
            }
            if (debug) MB("2");
            //*/
        }
        prev = cur;
    }

    CONS("Number of Anchors: %d\r\n", anchor_list.size());
    /*
    prev = anchor_list.front();
    int count = 0;
    foreach (vi, anchor_list, List<NPRVertex*>) {
        //(*vi)->m_index = count;
        count++;
        if (vi == anchor_list.begin()) continue;
        NPRVertex* cur = *vi;
        NPREdge* e = prev->getEdgeTo(cur);
        if (e) {
            e->count = m_drawerB.getMaxTol();
            if (e->twin())
                e->twin()->count = m_drawerB.getMaxTol();
        } else {
            if (prev != cur)
                CONS("Path broken.\r\n");
            //prev->addProp(VP_ANCHOR);
            //cur->addProp(VP_ANCHOR);
        }
        prev = cur;
    }
    */
#if 0
    FaceSet f_activeSet;
    SVertex* prev = &stroke.front();
    foreach (svi, stroke, Stroke) {
        SVertex* sv = &*svi;
        if (svi == stroke.begin()) continue;
        NPRFace* prev_f = getNearestPoint_slow(prev->vertex, mesh);
        NPRFace* f = getNearestPoint_slow(sv->vertex, mesh);
        if (prev_f == f) {
            prev = sv;
            continue;
        }
        dijkstraReset(&mesh);
        List<NPRFace*> f_list = faceDijkstraRun(prev_f, f);
        cutThroughFaces(mesh, f_list);
        /*
        NPRFace* f = NULL;
        if (svi == stroke.begin()) {
            f = getNearestPoint_slow(sv->vertex, mesh);
        } else {
            f = getNearestPoint(sv->vertex, f_activeSet);
        }
        splitFace(mesh, f, sv->vertex->nearestPoint, f_activeSet);
        */
        prev = sv;
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

List<NPRVertex*> CNPRAlg::cutThroughFaces(NPRMesh& mesh, List<NPRFace*>& f_list, Point3D& l1, Point3D& l2)
{
    List<NPRVertex*> result;
    List<NPRFace*> prev_f_list;
    prev_f_list.push_back(f_list.front());

    foreach (fi, f_list, List<NPRFace*>) {
        NPRFace* face = *fi;
        if (fi == f_list.begin()) {
            continue;
        }
        List<NPRFace*> candidates(prev_f_list);
        //CONS("candidate size: %d\r\n", candidates.size());
        prev_f_list.clear();
        //CONS("1\r\n");
        foreach (pfi, candidates, List<NPRFace*>) {
            NPRFace* pf = *pfi;
            //CONS("2\r\n");
            NPREdge* e = getCommonEdge(pf, face);
            //CONS("e: %d\r\n", e);
            if (e != NULL) {
                Point3D p = getNearestPoint(e->s()->p(), e->t()->p(), l1, l2, int(e->length()/m_tol));
                if ((p-e->s()->p()).abs() < m_tol && !e->s()->isProp(VP_ANCHOR)) {
                    result.push_back(e->s());
                    prev_f_list.push_back(face);
                    continue;
                } else if ((p-e->s()->p()).abs() < m_tol) {
                    p = e->s()->p()*0.9 + e->t()->p()*0.1;
                }
                if ((p-e->t()->p()).abs() < m_tol && !e->t()->isProp(VP_ANCHOR)) {
                    result.push_back(e->t());
                    prev_f_list.push_back(face);
                    continue;
                } else if ((p-e->t()->p()).abs() < m_tol) {
                    p = e->t()->p()*0.9 + e->s()->p()*0.1;
                }

                if (e->length() < epsilon()) {
                    CONS("stupid degenerated edge.\r\n");
                    prev_f_list.push_back(face);
                    continue;
                }

                //NPRVertex* s = e->s();
                //NPRVertex* t = e->t();
                //s->marked = true;
                //t->marked = true;
                NPRVertex* v = splitEdge(mesh, e, p);
                //NPRVertex* v = mesh.addVertex(p);
                result.push_back(v);
                //if (isnan(v->p().x()) || isnan(v->p().y()) || isnan(v->p().z())) {
                //    CONS("p=(%f %f %f)\r\n", p.x(), p.y(), p.z());
                //    CONS("s=(%f %f %f)\r\n", s->p().x(), s->p().y(), s->p().z());
                //    CONS("t=(%f %f %f)\r\n", t->p().x(), t->p().y(), t->p().z());
                //    CONS("v=(%f %f %f)\r\n", v->p().x(), v->p().y(), v->p().z());
                //    MB("trouble... Invalid point.");
                //}
                //s->marked = false;
                //t->marked = false;

                NPREdge::circulator ci = v->edge_circulator();
                do {
                    //CONS("?\r\n");
                    NPREdge* e = *ci;
                    prev_f_list.push_back(e->face());
                } while (++ci != v->edge_circulator() && ci != 0);
            }
        }
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPREdge* CNPRAlg::getCommonEdge(NPRFace* f1, NPRFace* f2)
{
    if (f1->e0()->twin()->face() == f2) return f1->e0();
    else if (f1->e1()->twin()->face() == f2) return f1->e1();
    else if (f1->e2()->twin()->face() == f2) return f1->e2();
    else return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPRVertex* CNPRAlg::splitFace(NPRMesh& mesh, NPRFace* f, Point3D p, CNPRAlg::FaceSet& f_set)
{
    f_set.clear();
    NPRVertex* v = mesh.addVertex(p);
    List<List<NPRVertex*> > v_list_list;
    NPREdge* e = f->e0();
    do {
        v_list_list.push_back();
        List<NPRVertex*>& v_list = v_list_list.back();
        v_list.push_back(e->s());
        v_list.push_back(e->t());
        v_list.push_back(v);
        e = e->nextInFace();
    } while (e != f->e0());

    mesh.removeFace(f);
    List<NPRFace*> f_list;
    foreach (vl, v_list_list, List<List<NPRVertex*> >) {
        NPRFace* face = mesh.addFace(*vl);
        f_list.push_back(face);
    }

    // update f_set
    foreach (fi, f_list, List<NPRFace*>) {
        NPRFace* f = *fi;
        NPREdge* e = f->e0();
        do {
            if (e->twin()) {
                f_set.insert(e->twin()->face());
            }
            e = e->nextInFace();
        } while (e != f->e0());
    }
    return v;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPRVertex* CNPRAlg::splitFace(NPRMesh& mesh, NPRFace* f, Point3D p)
{
    // check vertices
    NPREdge* e = f->e0();
    NPRVertex* min_v = NULL;
    real min_dist = bigval();
    do {
        real dist = (p-e->s()->p()).abs();
        if (!e->s()->isProp(VP_ANCHOR) && dist < m_tol && dist < min_dist) {
            min_dist = dist;
            min_v = e->s();
        }
        e = e->nextInFace();
    } while (e != f->e0());
    if (min_v != NULL) return min_v;

    // check edges
    NPREdge* min_e = NULL;
    min_dist = bigval();
    e = f->e0();
    do {
        real dist = e->distToEdge(p);
        if (dist < m_tol && dist < min_dist) {
            min_dist = dist;
            min_e = e;
        }
        e = e->nextInFace();
    } while (e != f->e0());
    if (min_e != NULL && min_e->count <= 0 && min_e->twin()->count <= 0) {
        //CONS("Edge\r\n");
        //min_e->count = 1;
        //mesh.refineEdge(min_e, v);
        return splitEdge(mesh, min_e, p);
    }

    // check faces
    NPRVertex* v = mesh.addVertex(p);
    List<List<NPRVertex*> > v_list_list;
    real vis_ratio = f->m_visRatio;
    e = f->e0();
    do {
        v_list_list.push_back();
        List<NPRVertex*>& v_list = v_list_list.back();
        v_list.push_back(e->s());
        v_list.push_back(e->t());
        v_list.push_back(v);
        e = e->nextInFace();
    } while (e != f->e0());

    mesh.removeFace(f);
    List<NPRFace*> f_list;
    foreach (vl, v_list_list, List<List<NPRVertex*> >) {
        NPRFace* face = mesh.addFace(*vl);
        // update visRatio
        face->m_visRatio = vis_ratio;
        f_list.push_back(face);
    }

    // update edge count
    NPREdge::circulator ci = v->edge_circulator();
    do {
        NPREdge* edge = (*ci)->nextInFace();
        if (edge->twin())
            edge->count = edge->twin()->count;
    } while (++ci != v->edge_circulator() && ci != 0);

    return v;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

int CNPRAlg::connectNearAnchors()
{
    int result = 0;
    int count = 10;
    //List<NPRVertex*> anchor_list;
    NPRMesh& mesh = getActiveMesh();
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        if (vi->isProp(VP_ANCHOR)) {
            NPRVertex* v = &*vi;
            NPRVertex* neighbor = NULL;
            int num_neighbor_anchor = getNearAnchors(v, neighbor);
            //if (v->m_index == 785) {
            //    CONS("num_neighbor_anchors: %d\r\n", num_neighbor_anchor);
            //}
            if (num_neighbor_anchor == 1) {
                // dijkstra search!
                dijkstraReset(&mesh);
                Point3D dir = (v->p() - neighbor->p()).unit();
                List<NPRVertex*> v_list = vertexDijkstraAnchor(v, 0.01, neighbor);
                if (v_list.empty())
                    v_list = vertexDijkstraAnchor(v, 0.05, dir);
                //foreach (va, v_list, List<NPRVertex*>) {
                //    (*va)->marked = true;
                //}
                if (v_list.empty()) continue;
                if (v_list.back() == neighbor) continue;
                Point3D s = v->p();
                Point3D t = v_list.back()->p();
                NPRVertex* prev = v;
                int v_count = 1;
                foreach (va, v_list, List<NPRVertex*>) {
                    NPREdge* edge = prev->getEdgeTo((*va));
                    if (edge != NULL) {
                        edge->count = 11;//count;//m_drawerB.getMaxTol();
                        if (edge->twin())
                            edge->twin()->count = 11;//count;//m_drawerB.getMaxTol();
                        count++;
                    }
                    if (!(*va)->isProp(VP_ANCHOR)) {
                        (*va)->addProp(VP_ANCHOR);
                        //anchor_list.push_back(*va);
                        real ratio = double(v_count)/double(v_list.size());
                        (*va)->p() = s*(1-ratio) + t*ratio;
                        result++;
                    }
                    prev = (*va);
                    v_count++;
                }
                //trim(prev, NULL, 2);
            }
        }
    }
    // smooth newly added anchors
    /*
    foreach (vi, anchor_list, List<NPRVertex*>) {
        NPRVertex* v = *vi;
        NPREdge::circulator ci = v->edge_circulator();
        Point3D pos;
        int num_neighbor_anchor = 0;
        do {
            NPRVertex* t = (*ci)->t();
            if (t->isProp(VP_ANCHOR)) {
                num_neighbor_anchor++;
                pos += t->p();
            }
        } while (++ci != v->edge_circulator() && ci != 0);
        pos /= num_neighbor_anchor;
        v->p() = pos;
    }
    */
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

int CNPRAlg::getNearAnchors(NPRVertex* v, NPRVertex*& neighbor)
{
    int count = 0;
    NPREdge::circulator ci = v->edge_circulator();
    do {
        NPREdge* e = *ci;
        NPRVertex* t = (*ci)->t();
        if (e->count > 0 || (e->twin() && e->twin()->count > 0)) {
            neighbor = t;
            count++;
        }
    } while (++ci != v->edge_circulator() && ci != 0);
    return count;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::trim(NPRMesh& mesh)
{
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = &*vi;
        if (v->isProp(VP_ANCHOR)) {
            int degree = featureDegree(v, v->edge_circulator());
            if (degree > 2) {
                //trim(v, NULL, 0.02);
                trim(v, 0.02);
                m_seedCount++;
                if (m_seedCount == 0) {
                    dijkstraReset(&mesh);
                    m_seedCount++;
                }
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::trim(NPRVertex* v, NPRVertex* prev, real depth)
{
    if (depth <= 0) {
        return false;
    }

    int count = 0;
    NPREdge::circulator ci = v->edge_circulator();
    do {
        NPREdge* e = *ci;
        real len = e->length();
        ASSERT(len > 0);
        if (e->count > 0 || (e->twin() && e->twin()->count > 0)) {
            NPRVertex* t = (*ci)->t();
            if (!t->isProp(VP_ANCHOR)) {
                t->marked = true;
                CONS("WARNING: non-anchor vertex on feature lines.\r\n");
            }
            count++;
            if (t != prev && trim(t, v, depth-len)) {
                e->count = 0;
                if (e->twin()) e->twin()->count = 0;
                t->remProp(VP_ANCHOR);
                //v->remProp(VP_ANCHOR);
                return true;
            }
        }
    } while (++ci != v->edge_circulator() && ci != 0);

    if (count == 1) {
        //v->remProp(VP_ANCHOR);
        return true;
    }
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::trim(NPRVertex* v, real distLeft)
{
    if (distLeft < 0) return false;
    v->seed = m_seedCount;
    int featureDegree = 0;
    NPREdge::circulator ci = v->edge_circulator();
    do {
        NPREdge* e = *ci;
        if (e->count > 0 || (e->twin() && e->twin()->count > 0)) {
            featureDegree++;
            if (e->t()->seed == m_seedCount) continue;
            NPRVertex* next_v = e->t();
            real len = e->length();
            if (trim(next_v, distLeft - len)) {
                e->count = 0;
                if (e->twin()) e->twin()->count = 0;
                next_v->remProp(VP_ANCHOR);
                return true;
            }
        }
    } while (++ci != v->edge_circulator() && ci != 0);
    if (featureDegree == 1) {
        return true;
    }
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::insertIntoQ(EdgeQueue& Q, NPREdge* e)
{
    real area = e->length();//e->face()->area();
    if (e->twin()) {
        //area += e->twin()->face()->area();
    }
    if (area < 0.01) {
        e->qIterator = Q.push(area, e);
        return true;
    } else {
        return false;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::removeFromQ(EdgeQueue& Q, NPREdge* e)
{
    if (e->qIterator != 0) {
        Q.erase(e->qIterator);
        e->qIterator = 0;
        return true;
    } else {
        return false;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPREdge* CNPRAlg::flipEdge(NPRMesh& mesh, NPREdge* e)
{
    ASSERT(e != NULL && e->twin());
    real vis_ratio1 = e->face()->m_visRatio;
    real vis_ratio2 = e->twin()->face()->m_visRatio;

    NPREdge* new_e = mesh.flipEdge(e);

    new_e->face()->m_visRatio = 0.5*(vis_ratio1 + vis_ratio2);
    foreach (ei, new_e->face()->edges(), NPRFace::Edges) {
        NPREdge* edge = *ei;
        if (!edge->twin()) continue;
        edge->count = edge->twin()->count;
    }
    if (new_e->twin()) {
        new_e->twin()->face()->m_visRatio = 0.5*(vis_ratio1 + vis_ratio2);
        foreach (ei, new_e->twin()->face()->edges(), NPRFace::Edges) {
            NPREdge* edge = *ei;
            if (!edge->twin()) continue;
            edge->count = edge->twin()->count;
        }
    }
    return new_e;
}
/////////////////////////////////////////////////////////////////////////////////////////
// not fully working
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::fixBadTriangles(NPRMesh& mesh, NPRVertex *v)
{
    NPREdge::circulator ci = v->edge_circulator();
    do {
        NPREdge* e = *ci;
        if (e->face()->largestAngle() > 160) {
        //if (e->face()->area() < epsilon()) {
            NPREdge* toFlip = NULL;
            real max_len = 0.0;
            do {
                real len = e->length();
                if (len > max_len) {
                    max_len = len;
                    toFlip = e;
                }
                e = e->nextInFace();
            } while (e != *ci);
            real angle = toFlip->dihedralAngle();
            if (angle > 60) continue;
            --ci;
            //toFlip->count = 1;
            CONS("angle: %f\r\n", e->face()->largestAngle());
            NPREdge* n_edge = mesh.flipEdge(toFlip);
            if (n_edge->face()->largestAngle() > 160) {
                mesh.flipEdge(n_edge);
            } else if (n_edge->twin() && n_edge->twin()->face()->largestAngle() > 160) {
                mesh.flipEdge(n_edge);
            }
            //return;
        }
        CONS(".");
    } while (++ci != v->edge_circulator() && ci != 0);
    CONS("done.\r\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::fixMesh(NPRMesh& mesh, bool preserveMode, bool preserveCrease, bool ifCollapseEdge, bool ifFlipEdge )
{
    if (ifCollapseEdge) {
        int count = collapseShortEdges(mesh, m_tol, preserveMode);
        CONS("%d edges collapsed.\r\n", count);
    }
    if (ifFlipEdge) {
        int count = flipEdges(mesh, preserveMode, preserveCrease);
        CONS("%d edges fliped.\r\n", count);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

List<NPRFace*> CNPRAlg::getRings(NPRVertex* v, int ring)
{
    List<NPRFace*> f_list;
    Heap<NPRFace*, int> Q;
    NPREdge::circulator ci = v->edge_circulator();
    do {
        NPRFace* f = (*ci)->face();
        f_list.push_back(f);
        f->m_ring = 1;
        Q.push(f, 1);
    } while (++ci != v->edge_circulator() && ci != 0);

    while (!Q.empty()) {
        NPRFace* f = Q.top_key();
        int cur_ring = f->m_ring;
        Q.pop();
        if (cur_ring == ring) continue;

        NPREdge* e = f->e0();
        do {
            NPREdge::circulator ci = e->s()->edge_circulator();
            do {
                NPRFace* face = (*ci)->face();
                if (face->m_ring == 0) {
                    face->m_ring = f->m_ring + 1;
                    Q.push(face, face->m_ring);
                    f_list.push_back(face);
                }
            } while (++ci != e->s()->edge_circulator() && ci != 0);
            e = e->nextInFace();
        } while (e != f->e0());
    }

    //foreach (fi, f_list, List<NPRFace*>) {
    //    (*fi)->m_ring = 0;
    //}
    return f_list;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

List<NPRVertex*> CNPRAlg::getNearByVertices(NPRMesh& mesh, NPRVertex* v, real radius)
{
    dijkstraReset(&mesh);
    List<NPRVertex*> v_list = vertexDijkstraRun(v, radius);

    return v_list;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

real CNPRAlg::gaussian(real x, real mu, real sig) const
{
	const real p = (x - mu)/sig;

	return pow(M_E, -p*p);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::onRidge(NPRVertex* v) const
{
    ASSERT(v != NULL);
    NPREdge::circulator ci = v->edge_circulator();
    do {
        NPREdge* edge = *ci;
        if (edge->dihedralAngle() > m_ridgeAngle) return true;
    } while (++ci != v->edge_circulator() && ci != 0);
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::refineMesh(NPRMesh& mesh)
{
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        ei->visited = false;
    }
    List<NPREdge*> edge_list;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* e = &*ei;
        if (e->visited) continue;
        if (e->dihedralAngle() > m_ridgeAngle) continue;
        NPRFace* f1 = (NPRFace*)(e->s()->mapped_face);
        NPRFace* f2 = (NPRFace*)(e->t()->mapped_face);
        if (f1 == NULL || f2 == NULL) {
            CONS("ERROR: please map before refine the mesh.\r\n");
            return;
        }
        Point3D n1 = f1->m_normal.unit();
        Point3D n2 = f2->m_normal.unit();
        real angle = acos(n1*n2)*180.0/M_PI;
        if (angle > m_ridgeAngle) {
            edge_list.push_back(e);
            e->visited = true;
            if (e->twin()) e->twin()->visited = true;
        }
    }

    foreach (ei, edge_list, List<NPREdge*>) {
        NPREdge* e = *ei;
        Point3D p = (e->s()->p() + e->t()->p())*0.5;
        NPRVertex* v = splitEdge(mesh, e, p);
        v->marked = true;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::refineMesh2(NPRMesh& mesh) {
    computeSolidVoxel(m_meshA);
    foreach (fi, m_meshA.faces(), NPRMesh::Faces) {
        fi->computePlaneEq();
    }
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        ei->visited = false;
    }
    List<NPREdge*> edge_list;
    List<Point3D> p_list;
    List<Point3D> mapped_list;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* e = &*ei;
        if (e->visited) continue;
        e->visited = true;
        if (e->twin()) e->twin()->visited = true;
        int num_samples = int(e->length()/m_tol);
        real max_error = 0;
        Point3D max_pos;
        Point3D max_mapped;
        for (int i=1; i<num_samples; i++) {
            real ratio = real(i)/real(num_samples);
            Point3D p = e->s()->p()*ratio + e->t()->p()*(1-ratio);
            Point3D mapped;
            getNearestPoint(p, m_meshA, mapped);
            real error = (mapped - p).abs();
            if (error > max_error) {
                max_error = error;
                max_pos = p;
                max_mapped = mapped;
            }
        }
        if (max_error > 3*m_tol) {
            edge_list.push_back(e);
            p_list.push_back(max_pos);
            mapped_list.push_back(max_mapped);
            /*
            NPRVertex* v = mesh.addVertex(max_pos);
            //NPRVertex* m = mesh.addVertex(max_mapped);
            v->marked = true;
            //m->marked = true;
            v->nearestPoint = max_mapped;
            MB("look");
            v->marked = false;
            //m->marked = false;
            */
        }
    }

    List<NPREdge*>::iterator ei=edge_list.begin();
    List<Point3D>::iterator pi=p_list.begin();
    List<Point3D>::iterator mi=mapped_list.begin();
    for (;ei != edge_list.end() && pi != p_list.end() && mi != mapped_list.end();
        ei++, pi++, mi++) {
            NPREdge* e = *ei;
            Point3D p = *pi;
            Point3D mapped = *mi;
            if (!e || !e->twin() || !e->twin()->twin() || e->twin()->twin() != e) {
                CONS("invalide edge.\r\n");
                continue;
            }
            NPRVertex* v = splitEdge(mesh, e, p);
            v->nearestPoint = mapped;
            //v->marked = true;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPREdge* CNPRAlg::nextFeatureEdge(NPREdge* edge) const
{
    int min_tol = m_drawerB.getMinTol();
    edge = edge->twin();
    ASSERT(edge != NULL);
    NPREdge::circulator begin(edge);
    NPREdge::circulator ci(begin);
    --ci;
    NPREdge* result = NULL;
    do {
        NPREdge* edge = *ci;
        if (edge->count > min_tol || (edge->twin() && edge->twin()->count > min_tol)) {
            result = edge;
            break;
        }
    } while (--ci != begin && ci != 0);
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

NPREdge* CNPRAlg::prevFeatureEdge(NPREdge* edge) const
{
    int min_tol = m_drawerB.getMinTol();
    NPREdge::circulator begin(edge);
    NPREdge::circulator ci(begin);
    ++ci;
    NPREdge* result = NULL;
    do {
        NPREdge* edge = *ci;
        if (edge->count > min_tol || (edge->twin() && edge->twin()->count > min_tol)) {
            result = edge;
            break;
        }
    } while (++ci != begin && ci != 0);
    if (result != NULL) result = result->twin();
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::smoothFeatureLineNormal(NPRMesh& mesh)
{
    int min_tol = m_drawerB.getMinTol();
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        if (edge->count > min_tol || (edge->twin() && edge->twin()->count > min_tol)) {
            NPREdge* next_feature = nextFeatureEdge(edge);
            NPREdge* prev_feature = prevFeatureEdge(edge);
            int count = 0;
            if (edge->normal.abs() > epsilon()) count++;
            if (next_feature != NULL
                && next_feature != edge->twin()
                && next_feature->normal.abs() > epsilon()) {
                    edge->normal += next_feature->normal;
                    count++;
            }
            if (prev_feature != NULL
                && prev_feature != edge->twin()
                && prev_feature->normal.abs() > epsilon()) {
                    edge->normal += prev_feature->normal;
                    count++;
            }

            if (count != 0) {
                edge->normal.normalize();
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::assignAnchorNormal(NPRMesh& mesh)
{
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = &*vi;
        if (!v->isProp(VP_ANCHOR)) continue;
        v->m_normals.clear();
        NPREdge::circulator ci = v->edge_circulator();
        NPREdge::circulator begin = ci;
        int degree = featureDegree(v, begin);
        if (degree == 0) continue;
        if (degree == 1) {
            NPREdge* e = *begin;
            ci = begin;
            NPREdge::circulator prev = ci;
            ++ci;
            real angle = 0.0;
            Color3d c1 = getRandomColor();
            Color3d c2 = getRandomColor();
            do {
                Point3D v1 = (*prev)->t()->p() - (*prev)->s()->p();
                Point3D v2 = (*ci)->t()->p() - (*ci)->s()->p();
                if (v1.abs() > epsilon() && v2.abs() > epsilon()) {
                    v1.normalize();
                    v2.normalize();
                    real d_angle = acos(v1*v2)*180.0/M_PI;
                    if (angle + d_angle/2.0 < 180) {
                        v->m_normals[(*prev)->face()] = e->normal;
                        v->m_normal_colors[(*prev)->face()] = c1;
                    } else {
                        v->m_normals[(*prev)->face()] = e->twin()->normal;
                        v->m_normal_colors[(*prev)->face()] = c2;
                    }
                    angle += d_angle;
                }
                prev = ci;
                ++ci;
            } while (prev != begin && ci != 0);
            /*
            Color3d c(rand(), rand(), rand());
            c /= RAND_MAX;
            v->m_normals[e->face()] = e->normal;
            v->m_normal_colors[e->face()] = c;
            c = Color3d(rand(), rand(), rand());
            c /= RAND_MAX;
            v->m_normals[e->twin()->face()] = e->twin()->normal;
            v->m_normal_colors[e->twin()->face()] = c;
            */
            continue;;
        }

        if ((*begin)->count > 0 || ((*begin)->twin() && (*begin)->twin()->count > 0)) {
            List<NPRFace*> f_list;
            Point3D normal = (*begin)->normal;
            ci = begin;
            do {
                NPREdge* e = *ci;
                NPRFace* f = e->face();
                ASSERT(e->twin());
                if (e->count > 0 || e->twin()->count > 0) {
                    normal += e->twin()->normal;
                    normal.normalize();
                    Color3d c(rand(), rand(), rand());
                    c /= RAND_MAX;
                    foreach (fi, f_list, List<NPRFace*>) {
                        v->m_normals[*fi] = normal;
                        v->m_normal_colors[*fi] = c;
                    }
                    normal = e->normal;
                    f_list.clear();
                }
                f_list.push_back(f);
            } while (++ci != begin && ci != 0);
            // process the last sector
            normal += (*begin)->twin()->normal;
            normal.normalize();
            Color3d c(rand(), rand(), rand());
            c /= RAND_MAX;
            foreach (fi, f_list, List<NPRFace*>) {
                v->m_normals[*fi] = normal;
                v->m_normal_colors[*fi] = c;
            }
            f_list.clear();
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

int CNPRAlg::featureDegree(const NPRVertex* v, NPREdge::circulator& begin) const
{
    int min_tol = m_drawerB.getMinTol();
    int count = 0;
    NPREdge::circulator ci = v->edge_circulator();
    do {
        NPREdge* e = *ci;
        if (e->count > min_tol || (e->twin() && e->twin()->count > min_tol)) {
            begin = ci;
            count++;
        }
    } while (++ci != v->edge_circulator() && ci != 0);
    return count;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::removeIsolatedAnchors(NPRMesh& mesh)
{
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = &*vi;
        if (v->isProp(VP_ANCHOR)) {
            NPRVertex* dummy = NULL;
            int num_near_anchors = getNearAnchors(v, dummy);
            if (num_near_anchors == 0) {
                v->remProp(VP_ANCHOR);
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::cutThinTriangles(NPRMesh& mesh, bool preserveMode)
{
    EdgeQueue Q;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        if (preserveMode) {
            if (edge->count > 0 || (edge->twin() && edge->twin()->count > 0)) {
                continue;
            }
        }
        real angle = edge->oppAngle();
        real angle2 = edge->twin()->oppAngle();
        edge->qIterator = 0;
        if (isnan(angle)) {
            edge->count = 2;
            continue;
        }
        if (angle < 120.0) continue;
        //if (angle + angle2 > 181.0) continue; // flipping will work
        edge->qIterator = Q.push(180-angle, edge);
    }

    int num_faces = mesh.nFaces();
    while (!Q.empty()) {
        NPREdge* edge = Q.top_value();
        Point3D s = edge->s()->p();
        Point3D t = edge->t()->p();
        Point3D p = edge->nextInFace()->t()->p();
        if ((p-s).abs() < epsilon() || (p-t).abs() < epsilon()) {
            Q.pop();
            edge->qIterator = 0;
            continue;
        }

        //edge->count = 180;
        real angle = Q.top_key();
        //CONS("angle: %f\r\n", angle);
        //MB("Try your best, and you can do it!");
        NPREdge* e = edge;
        do {
            if (e->qIterator != 0) {
                Q.erase(e->qIterator);
                e->qIterator = 0;
            }
            e = e->nextInFace();
        } while (e != edge);
        if (edge->twin()) {
            e = edge->twin();
            do {
                if (e->qIterator != 0) {
                    Q.erase(e->qIterator);
                    e->qIterator = 0;
                }
                e = e->nextInFace();
            } while (e != edge->twin());
        }

        NPRVertex* nv = splitEdge(mesh, e, p, false);

        NPREdge::circulator ci = nv->edge_circulator();
        do {
            NPREdge* ne = *ci;
            ne->face()->color() = Color3d(0.5, 0.5, 1.0);
            e = ne;
            do {
                real angle = e->oppAngle();
                real angle2 = e->twin()->oppAngle();
                if (e->qIterator != 0) {
                    Q.erase(e->qIterator);
                    e->qIterator = 0;
                }
                if (angle >= 120 /*&& (angle + angle2) <= 181.0*/) {
                    e->qIterator = Q.push(180-angle, e);
                }
                e = e->nextInFace();
            } while (e != ne);
        } while (++ci != nv->edge_circulator() && ci != 0);
    }
    CONS("old: %d  new: %d\r\n", num_faces, mesh.nFaces());
}

/////////////////////////////////////////////////////////////////////////////////////////
// Collapse all edges of lenght < tol, and preserves the ridge
// if preserveMode, feature lines will remain untouched.
/////////////////////////////////////////////////////////////////////////////////////////

int CNPRAlg::collapseShortEdges(NPRMesh& mesh, real tol, bool preserveMode)
{
    //m_mutex.lock();
    real area_tol = mesh.getAveFaceArea()/10.0;
    EdgeQueue Q;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        edge->qIterator = 0;
        if (edge->length() < tol) {
            edge->qIterator = Q.push(edge->length(), edge);
            if (edge->debug) {
                CONS("Inserted\r\n");
            }
        }
    }

    int count = 0; // count number of collapsed edges
    while (!Q.empty()) {
        NPREdge* e = Q.top_value();

        if (preserveMode) {
            NPREdge::circulator ci = e->s()->edge_circulator();
            bool isFeature = false;
            do {
                NPREdge* edge = *ci;
                if (edge->count > 0 || (edge->twin() && edge->twin()->count > 0)) {
                    Q.pop();
                    e->qIterator = 0;
                    isFeature = true;
                    break;
                }
            } while (++ci != e->s()->edge_circulator() && ci != 0);
            if (isFeature && e->debug) {
                CONS("filtered as feature\r\n");
            }
            if (isFeature) continue;
        }

        real s_angles = 0.0, t_angles = 0.0;
        NPREdge* temp = e;
        do {
            real angle = temp->dihedralAngle();
            if (angle > 0.0)
                s_angles += angle;
            temp = temp->nextOfS();
        } while (temp != e);
        temp = e->twin();
        do {
            real angle  = temp->dihedralAngle();
            if (angle > 0.0)
                t_angles += angle;
            temp = temp->nextOfS();
        } while (temp != e->twin());

        if (s_angles > t_angles + epsilon()) {
            if (e->debug) {
                CONS("filtered because source has bigger curvature than target\r\n");
            }
            Q.pop();
            e->qIterator = 0;
            continue;
        }

        /*
        if (e->length() > m_tol // Long enough
            && e->dihedralAngle() <= m_ridgeAngle // itself is not a ridge
            && s_angles > t_angles + epsilon()
            //&& onRidge(e->s()) // but s is on a ridge
            //&& e->face()->area() > area_tol // face is not degenerated
            //&& e->twin()
            //&& e->twin()->face()->area() > area_tol // face is not degenerated
            ) {
                Q.pop();
                e->qIterator = 0;
                if (e->debug) {
                    CONS("length: %f > %f\r\n", e->length(), epsilon());
                    CONS("di angle %f <= %f\r\n", e->dihedralAngle(), m_ridgeAngle);
                    CONS("onRidge %d\r\n", onRidge(e->s()));
                    CONS("Area 1 %f > %f\r\n", e->face()->area(), epsilon());
                    CONS("Area 2 %f > %f\r\n", e->twin()->face()->area(), epsilon());
                    CONS("filtered because collapsing will destroy a ridge...\r\n");
                }
                continue;
        }
        */

        if (!mesh.collapsedCheck(e)) { // check for 3-loops
            if (e->debug) {
                CONS("filtered due because it forms a 3-loop.\r\n");
            }
            Q.pop();
            e->qIterator = 0;
            continue;
        }

        NPRVertex* t = e->t();

        NPREdge::circulator ci = e->s()->edge_circulator();
        do {
            NPREdge* edge = *ci;
            if (edge->qIterator != 0) {
                Q.erase(edge->qIterator);
                edge->qIterator = 0;
            }
            if (edge->twin()->qIterator != 0) {
                Q.erase(edge->twin()->qIterator);
                edge->twin()->qIterator = 0;
            }
        } while (++ci != e->s()->edge_circulator() && ci != 0);
        ci = e->t()->edge_circulator();
        do {
            NPREdge* edge = *ci;
            if (edge->qIterator != 0) {
                Q.erase(edge->qIterator);
                edge->qIterator = 0;
            }
            if (edge->twin()->qIterator != 0) {
                Q.erase(edge->twin()->qIterator);
                edge->twin()->qIterator = 0;
            }
        } while (++ci != e->t()->edge_circulator() && ci != 0);

        //if (e->debug || e->twin()->debug) {
            //e->count = 100;
            //e->s()->marked = true;
            //m_mutex.unlock();
            //MB("i");
            //m_mutex.lock();
            //e->count = 0;
        //}
        mesh.collapseEdge(e);
        count++;

        ci = t->edge_circulator();
        do {
            NPREdge* edge = *ci;
            if (edge->length() < m_tol) {
                edge->qIterator = Q.push(edge->length(), edge);
            }
            if (edge->twin()) {
                int c = max(edge->count, edge->twin()->count);
                edge->count = edge->twin()->count = c;
            }
        } while (++ci != t->edge_circulator() && ci != 0);
    }
    //m_mutex.unlock();
    return count;

    /*
    EdgeQueue Q;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        edge->qIterator = 0;
        if (edge->dihedralAngle() < m_ridgeAngle && onRidge(edge->s()))
            continue;
        if (edge->length() < m_tol) {
            edge->qIterator = Q.push(edge->length(), edge);
        }
    }

    while (!Q.empty()) {
        NPREdge* edge = Q.top_value();
        NPREdge* e = edge;
        do {
            if (e->qIterator != 0) {
                Q.erase(e->qIterator);
                e->qIterator = 0;
            }
            e = e->nextInFace();
        } while (e != edge);
        if (edge->twin()) {
            e = edge->twin();
            do {
                if (e->qIterator != 0) {
                    Q.erase(e->qIterator);
                    e->qIterator = 0;
                }
                e = e->nextInFace();
            } while (e != edge->twin());
        }

        if (mesh.collapsedCheck(e))
            mesh.collapseEdge(e);
    }
    */
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

int CNPRAlg::flipEdges(NPRMesh& mesh, bool preserveMode, bool preserveCrease)
{
    EdgeQueue Q;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* e = &*ei;
        e->qIterator = 0;
        if (!e->twin()) continue;
        real angle1 = e->oppAngle();
        real angle2 = e->twin()->oppAngle();
        if (angle1 < -epsilon() || angle2 < -epsilon()) continue;
        if (angle1 + angle2 > 181.0) {
            EdgeQueue::iterator itr = Q.push(360-angle1-angle2, e);
            e->qIterator = itr;
        }
    }

    int count = 0;
    while (!Q.empty()) {
        NPREdge* e = Q.top_value();
        if (preserveMode &&
            (e->count > 0 || e->twin() && e->twin()->count > 0)) {
            Q.pop();
            e->qIterator = 0;
            continue;
        }

        if ( preserveCrease ) {
            NPRVertex* s = e->s();
            NPRVertex* t = e->t();
            NPRVertex* v0 = e->nextInFace()->t();
            NPRVertex* v1 = e->twin()->nextInFace()->t();
            if (!smoothCheck(v0->p(), s->p(), v1->p(), v0->normal, s->normal, v1->normal, m_smooth_tol)
										|| 
				!smoothCheck(v0->p(), v1->p(), t->p(), v0->normal, v1->normal, t->normal, m_smooth_tol)) {
                Q.pop();
                e->qIterator = 0;
                continue;
            }
        }

        NPREdge* test = e->nextInFace()->t()->getEdgeTo(
            e->twin()->nextInFace()->t());
        if (test != NULL) {
            Q.pop();
            e->qIterator = 0;
            continue;
        }

        foreach (ei, e->face()->edges(), NPRFace::Edges) {
            NPREdge* edge = *ei;
            if (edge->qIterator != 0) {
                Q.erase(edge->qIterator);
                edge->qIterator = 0;
            }
            if (edge->twin()->qIterator != 0) {
                Q.erase(edge->twin()->qIterator);
                edge->twin()->qIterator = 0;
            }
        }
        if (e->twin()) {
            foreach (ei, e->twin()->face()->edges(), NPRFace::Edges) {
                NPREdge* edge = *ei;
                if (edge->qIterator != 0) {
                    Q.erase(edge->qIterator);
                    edge->qIterator = 0;
                }
                if (edge->twin()->qIterator != 0) {
                    Q.erase(edge->twin()->qIterator);
                    edge->twin()->qIterator = 0;
                }
            }
        }

        ASSERT(e->s() == e->twin()->t());
        ASSERT(e->t() == e->twin()->s());

        NPREdge* new_e = flipEdge(mesh, e);
        count++;
        new_e->face()->color() = Color3d(0.0, 0.0, 1.0);
        new_e->twin()->face()->color() = Color3d(0.0, 0.0, 1.0);

        foreach (ei, new_e->face()->edges(), NPRFace::Edges) {
            NPREdge* edge = *ei;
            edge->s()->updateNormal();
            if (!edge->twin()) continue;
            //edge->count = edge->twin()->count;
            real angle1 = edge->oppAngle();
            real angle2 = edge->twin()->oppAngle();
            if (angle1 < -epsilon() || angle2 < -epsilon()) continue;
            if (angle1 + angle2 > 181.0) {
                EdgeQueue::iterator itr = Q.push(360-angle1-angle2, edge);
                edge->qIterator = itr;
            }
        }
        if (new_e->twin()) {
            foreach (ei, new_e->twin()->face()->edges(), NPRFace::Edges) {
                NPREdge* edge = *ei;
                edge->s()->updateNormal();
                if (!edge->twin()) continue;
                //edge->count = edge->twin()->count;
                real angle1 = edge->oppAngle();
                real angle2 = edge->twin()->oppAngle();
                if (angle1 < -epsilon() || angle2 < -epsilon()) continue;
                if (angle1 + angle2 > 181.0) {
                    EdgeQueue::iterator itr = Q.push(360-angle1-angle2, edge);
                    edge->qIterator = itr;
                }
            }
        }
    }

    return count;

    /*
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        if (!ei->twin()) continue;
        if (ei->count > 0 || ei->twin()->count > 0) continue;
        if (ei->face()->area() < epsilon() || ei->twin()->face()->area() < epsilon()) {
            NPREdge* edge = flipEdge(mesh, &*ei);
            NPREdge* e = edge;
            do {
                if (e->twin()) {
                    e->count = e->twin()->count;
                }
                e = e->nextInFace();
            } while (e != edge);
            e = edge->twin();
            do {
                if (e->twin()) {
                    e->count = e->twin()->count;
                }
                e = e->nextInFace();
            } while (e != edge->twin());
        }
    }
    */
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::registerNormals(NPRMesh& mesh)
{
    foreach_const (ei, mesh.edges(), NPRMesh::Edges) {
        const NPREdge* e = &*ei;
        if (e->count > 0 || (e->twin() && e->twin()->count > 0)) {
            m_normalMap[std::pair<NPRVertex*, NPRVertex*>(e->s(), e->t())] =
                e->normal;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::resampleStrokes(NPRMesh& mesh)
{
    Array<bool> keep(mesh.nVertices(), false);
    Array<int> t_junction(mesh.nVertices(), 0);
    // Keep the end points and corners
    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        SVertex* prev = NULL;
        SVertex* pprev = NULL;
        foreach (svi, (*stroke), Stroke) {
            SVertex* sv = &*svi;
            int index = sv->vertex->m_index;
            t_junction[index]++;
            if (sv == &(stroke->front()) || sv == &(stroke->back())) {
                keep[index] = true;
            }
            if (prev != NULL && pprev != NULL) {
                Point3D v1 = (pprev->vertex->p() - prev->vertex->p()).unit();
                Point3D v2 = (sv->vertex->p() - prev->vertex->p()).unit();
                real angle = acos(v1*v2)*180.0/M_PI;
                if (angle < 150) keep[prev->vertex->m_index] = true;
            }
            pprev = prev;
            prev = sv;
        }
    }

    // Keep the T-junctions
    for (int i=0; i<t_junction.size(); i++) {
        if (t_junction[i] > 1)
            keep[i] = true;
    }

    ///*
    // Max error sampling
    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        SVertex* prev = NULL;
        List<SVertex*> sv_list;
        foreach (svi, (*stroke), Stroke) {
            SVertex* sv = &*svi;
            int index = sv->vertex->m_index;
            if (prev == NULL && keep[index]) {
                prev = sv;
                continue;
            }

            sv_list.push_back(sv);
            if (keep[index]) {
                maxErrorSampling(sv_list, sv_list.begin(),
                    sv_list.end(), keep, m_tol);
                //real max_err = 0.0;
                //int max_index = index;
                //foreach (vi, sv_list, List<SVertex*>) {
                //    NPRVertex* v_between = (*vi)->vertex;
                //    real err = p2lDist(v_between->p(),
                //        prev->vertex->p(), sv->vertex->p());
                //    if (err > max_err) {
                //        max_err = err;
                //        max_index = v_between->m_index;
                //    }
                //}
                //if (max_err> m_tol) {
                //    keep[max_index] = true;
                //}
                prev = sv;
                sv_list.clear();
            }
        }
    }
    //*/

    // Merge points too close to one another
    ///*
    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        SVertex* prev = NULL;
        List<SVertex*> sv_list;
        foreach (svi, (*stroke), Stroke) {
            SVertex* sv = &*svi;
            int index = sv->vertex->m_index;
            if (prev == NULL && keep[index]) {
                prev = sv;
                continue;
            }

            sv_list.push_back(sv);
            if (keep[index]) {
                real dist = (sv->vertex->p() - prev->vertex->p()).abs();
                if (dist < 2*m_tol) {
                    Point3D p = (sv->vertex->p() + prev->vertex->p())*0.5;
                    prev->vertex->p() = p;
                    //sv->vertex = prev->vertex;
                    foreach (svii, sv_list, List<SVertex*>) {
                        SVertex* sv_between = *svii;
                        sv_between->vertex = prev->vertex;
                        sv_between->vertex->p() = p;
                    }
                }
                prev = sv;
                sv_list.clear();
            }
        }
    }
    //*/

    ///*
    // Break long line segments
    real sample_dist = 7 * getAveStrokeDist();
    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        SVertex* prev = NULL;
        real length = 0.0;
        foreach (svi, (*stroke), Stroke) {
            SVertex* sv = &*svi;
            int index = sv->vertex->m_index;
            if (prev == NULL && keep[index]) {
                prev = sv;
                continue;
            }

            real dist = (sv->vertex->p() - prev->vertex->p()).abs();
            length += dist;
            if (keep[index]) {
                length = 0.0;
            } else if (length - dist < 0.5*sample_dist && length > sample_dist) {
                keep[index] = true;
                length = 0.0;
            } else if (length > sample_dist) {
                keep[prev->vertex->m_index] = true;
                length = dist;
            }
            prev = sv;
        }
    }
    //*/

    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        foreach (svi, (*stroke), Stroke) {
            SVertex* sv = &*svi;
            int index = sv->vertex->m_index;
            if (!keep[index]) {
                Stroke::iterator prev = svi.prev();
                stroke->erase(svi);
                svi = prev;
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::maxErrorSampling(List<SVertex*>& sv_list,
        List<SVertex*>::iterator start,
        List<SVertex*>::iterator end,
        Array<bool>& keep, real err_tol)
{
    SVertex* first = *start;
    SVertex* last;
    if (end == sv_list.end()) last = sv_list.back();
    else last = *end;
    List<SVertex*>::iterator svi;
    List<SVertex*>::iterator max_vertex;
    real max_err = 0.0;
    for (svi = start; svi != end; svi++) {
        NPRVertex* v_between = (*svi)->vertex;
        real err = p2lDist(v_between->p(),
            first->vertex->p(), last->vertex->p());
        if (err > max_err) {
            max_err = err;
            max_vertex = svi;
        }
    }

    if (max_err > err_tol) {
        keep[(*max_vertex)->vertex->m_index] = true;
        maxErrorSampling(sv_list, start, max_vertex, keep, err_tol);
        maxErrorSampling(sv_list, max_vertex, end, keep, err_tol);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

real CNPRAlg::pathDist(const List<NPRVertex*>& v_list) const
{
    real dist = 0.0;
    NPRVertex* prev = NULL;
    foreach_const(vi, v_list, List<NPRVertex*>) {
        NPRVertex* v = *vi;
        if (v == v_list.front()) {
            prev = v;
            continue;
        }
        dist += (v->p() - prev->p()).abs();
        prev = v;
    }
    return dist;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::filterAnchors(NPRMesh& mesh, real angle_tol)
{
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = &*vi;
        if (!v->isProp(VP_ANCHOR)) continue;
        List<Point3D> normals;
        NPREdge::circulator ci = v->edge_circulator();
        do {
            NPRFace* f = (*ci)->face();
            std::map<void*, Point3D>::iterator itr =
                v->m_normals.find(f);
            if (itr == v->m_normals.end()) continue;
            Point3D n = itr->second;
            bool exist = false;
            foreach (ni, normals, List<Point3D>) {
                Point3D normal = *ni;
                if ((normal - n).abs() < epsilon()) {
                    exist = true;
                    break;
                }
            }
            if (!exist) {
                normals.push_back(n);
            }
        } while (++ci != v->edge_circulator() && ci != 0);
        if (normals.size() == 0) continue;

        real largest_angle = 0.0;
        for (List<Point3D>::iterator ni = normals.begin();
            ni != normals.end(); ni++) {
                Point3D n1 = *ni;
                List<Point3D>::iterator noi = ni;
                noi++;
                for (; noi != normals.end(); noi++) {
                    Point3D n2 = *noi;
                    real angle = acos(n1*n2)*180.0/M_PI;
                    if (angle > largest_angle) largest_angle = angle;
                }
        }
        if (largest_angle < angle_tol) {
            //v->marked = true;
            ///*
            v->remProp(VP_ANCHOR);
            NPREdge::circulator ci = v->edge_circulator();
            do {
                NPREdge* e = *ci;
                e->count = 0;
                if (e->twin()) {
                    e->twin()->count = 0;
                }
            } while (++ci != v->edge_circulator() && ci != 0);
            //*/
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::restoreAnchors(NPRMesh& mesh)
{
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        if (edge->count > 0) {
            edge->s()->addProp(VP_ANCHOR);
            edge->t()->addProp(VP_ANCHOR);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::isValidMesh(const NPRMesh& mesh) const
{
    foreach_const (vi, mesh.vertices(), NPRMesh::Vertices) {
        const NPRVertex* v = &*vi;
        if (isnan(v->p().x()) || isnan(v->p().y()) || isnan(v->p().z())) {
            v->marked = true;
            return false;
        }
    }
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::computeDistToFeature(NPRMesh& mesh, bool coloring)
{
    int num_f = mesh.nFaces();
    Heap<real, NPRFace*, greater<real>> Q;

    // initialize the 1-ring neighbors of feature lines to 1,
    // and everything else to -1.
    foreach (fi, mesh.faces(), NPRMesh::Faces) {
        NPRFace* face = &*fi;
        real dist = bigval();
        const Point3D c = face->centroid();
        foreach_const (ei, face->edges(), NPRFace::Edges) {
            const NPREdge* e = *ei;
            if (e->count > 0 || (e->twin() && e->twin()->count > 0)) {
                real d = e->distToEdge(c);
                if (d < dist) dist = d;
            }
        }
        if (dist < bigval() - epsilon()) {
            Q.push(dist, face);
            face->m_dist = bigval();
        } else {
            face->m_dist = bigval();
        }
    }

    while (!Q.empty()) {
        NPRFace* face = Q.top_value();
        real dist = Q.top_key();
        Q.pop();
        if (dist < face->m_dist)
            face->m_dist = dist;
        else continue;

        foreach_const (ei, face->edges(), NPRFace::Edges) {
            const NPREdge* e = *ei;
            if (!e->twin()) continue;
            if (e->count > 0 || e->twin()->count > 0) continue;
            NPRFace* f = e->twin()->face();
            real addDist = (f->centroid() - face->centroid()).abs();
            if (dist + addDist < f->m_dist) {
                Q.push(dist + addDist, f);
            }
        }
    }

    if (!coloring) return;

    foreach (fi, mesh.faces(), NPRMesh::Faces) {
        real c = fi->m_dist * 2.0;
        c = min(c, 1.0);
        c = max(c, 0.0);
        fi->color() = Color3d(c, 0, 0);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Extract faces that are furthest away from the feature lines.
/////////////////////////////////////////////////////////////////////////////////////////

List<NPRFace*> CNPRAlg::extractLocalMax(const NPRMesh& mesh) const
{
    List<NPRFace*> result;
    foreach_const (fi, mesh.faces(), NPRMesh::Faces) {
        const NPRFace* face = &*fi;
        bool localMax = true;
        foreach_const (ei, face->edges(), NPRFace::Edges) {
            NPREdge* e = const_cast<NPREdge*>(*ei);
            NPRVertex* v= e->s();
            NPREdge::circulator ci(e);
            do {
                NPRFace* f = (*ci)->face();
                if (f == face) continue;
                if (f->m_dist > face->m_dist) {
                    localMax = false;
                }
            } while (++ci != e && ci != 0);
            if (!localMax) break;
        }
        if (localMax) result.push_back(const_cast<NPRFace*>(face));
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::partition(NPRMesh& mesh, List<NPRFace*>& seeds)
{
    std::map<NPRFace*, int> chart_map;
    Array<Point3D> chart_normal(seeds.size());
    Array<real> chart_area(seeds.size(), 0.0);
    Array<List<NPRFace*> > charts(seeds.size());
    Heap<real, NPRFace*, greater<real>> Q;

    foreach (fi, mesh.faces(), NPRMesh::Faces) {
        NPRFace* face = &*fi;
        chart_map[face] = -1;
    }

    int count = 0;
    foreach (fi, seeds, List<NPRFace*>) {
        NPRFace* face = *fi;
        chart_map[face] = count;
        chart_normal[count] = face->m_normal;
        chart_area[count] = face->m_area;
        charts[count].push_back(face);
        foreach (ei, face->edges(), NPRFace::Edges) {
            NPREdge* edge = *ei;
            if (!edge->twin()) continue;
            real cost = 1.0 - face->m_normal * edge->twin()->face()->m_normal;
            Q.push(cost, edge->twin()->face());
        }
        count++;
    }

    while (!Q.empty()) {
        NPRFace* face = Q.top_value();
        Q.pop();

        int min_chart = -1;
        real min_cost = bigval();
        foreach (ei, face->edges(), NPRFace::Edges) {
            NPREdge* edge = *ei;
            if (!edge->twin()) continue;
            if (edge->count > 0 || edge->twin()->count > 0) continue;
            if (chart_map[edge->twin()->face()] < 0) continue;
            int chart_index = chart_map[edge->twin()->face()];
            real cost = 1.0 - face->m_normal * chart_normal[chart_index];
            if (cost < min_cost) {
                min_cost = cost;
                min_chart = chart_index;
            }
        }
        ASSERT(min_chart != -1);
        /*
        if (min_chart == -1) {
            foreach (ei, face->edges(), NPRFace::Edges) {
                NPREdge* edge = *ei;
                CONS("edge->twin() == %d\r\n", edge->twin());
                CONS("edge->count = %d  edge->twin()->count = %d\r\n",
                    edge->count, edge->twin()->count);
                CONS("chart index: %d\r\n", chart_map[edge->twin()->face()]);
                int chart_index = chart_map[edge->twin()->face()];
                real cost = 1.0 - face->m_normal * chart_normal[chart_index];
                CONS("cost = %f\r\n", cost);
                CONS("face normal: %f %f %f\r\n",
                    face->m_normal.x(), face->m_normal.y(), face->m_normal.z());
                CONS("chart normal: %f %f %f\r\n",
                    chart_normal[chart_index].x(), chart_normal[chart_index].y(), chart_normal[chart_index].z());
            }
            face->color() = Color3d(0.0, 0.0, 1.0);
            MB("Error, Isolated face.");
        }
        */
        chart_map[face] = min_chart;
        chart_normal[min_chart] =
            chart_normal[min_chart]*chart_area[min_chart] +
            face->m_normal*face->m_area;
        chart_normal[min_chart].normalize();
        chart_area[min_chart] += face->m_area;
        charts[min_chart].push_back(face);
        face->color() = Color3d(1.0, 0.0, 0.0);

        foreach (ei, face->edges(), NPRFace::Edges) {
            NPREdge* edge = *ei;
            if (!edge->twin()) continue;
            if (edge->count > 0 || edge->twin()->count > 0) continue;
            if (chart_map[edge->twin()->face()] >= 0) {
                int n_chart_index = chart_map[edge->twin()->face()];
                if (n_chart_index != min_chart &&
                    acos(chart_normal[n_chart_index]*chart_normal[min_chart])*180/M_PI < m_angleTol) {
                        // merge 2 charts
                        while (!charts[n_chart_index].empty()) {
                            NPRFace* f = charts[n_chart_index].front();
                            charts[n_chart_index].pop_front();
                            charts[min_chart].push_back(f);
                            chart_map[f] = min_chart;
                        }
                        chart_normal[min_chart] = chart_normal[min_chart]*chart_area[min_chart] +
                            chart_normal[n_chart_index]*chart_area[n_chart_index];
                        chart_normal[min_chart].normalize();
                        chart_area[min_chart] += chart_area[n_chart_index];
                        chart_area[n_chart_index] = 0.0;
                }
                continue;
            }
            real cost = 1.0 - edge->twin()->face()->m_normal * chart_normal[min_chart];
            //real cost = 1.0 - edge->twin()->face()->m_normal * face->m_normal;
            Q.push(cost, edge->twin()->face());
        }
    }

    // coloring
    foreach (f_list_i, charts, Array<List<NPRFace*> >) {
        Color3d c = getRandomColor();
        foreach (fi, (*f_list_i), List<NPRFace*>) {
            NPRFace* face = *fi;
            face->color() = c;
        }
    }
    // add extra feature edges
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        if (!edge->twin()) continue;
        if (chart_map[edge->face()] != chart_map[edge->twin()->face()])
            edge->count = max(edge->count, 1);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

Color3d CNPRAlg::getRandomColor() const
{
    Color3d c(rand(), rand(), rand());
    c /= RAND_MAX;
    return c;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Resample the feature lines so that the dist between 2 sample points is >= sampleDist
// but < 2*sampleDist.
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::resampleFeatureLines(NPRMesh& mesh, real sampleDist)
{
    EdgeQueue Q;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        edge->qIterator = 0;
        if (edge->length() >= sampleDist) continue;
        if (edge->count > 0 || (edge->twin() && edge->twin()->count > 0)) {
            edge->qIterator = Q.push(edge->length(), edge);
        }
    }

    int count = 0;
    while (!Q.empty()) {
        NPREdge* e = Q.top_value();
        /*
        CONS("%d\r\n", Q.size());
        count++;
        if (count > 300) {
            e->s()->marked = true;
            e->t()->marked = true;
            e->debug = true;
            MB("...");
            e->s()->marked = false;
            e->t()->marked = false;
        }
        */

        if (!e->twin()) {
            Q.pop();
            e->qIterator = 0;
            if (e->debug)
                CONS("Removed: no twin.\r\n");
            continue;
        }

        if (e->count <= 0 && e->twin()->count <= 0) {
            Q.pop();
            e->qIterator = 0;
            if (e->debug)
                CONS("Removed: not feature edge.\r\n");
            continue;
        }

        if (!mesh.collapsedCheck(e)) {
            Q.pop();
            e->qIterator = 0;
            if (e->debug)
                CONS("Removed: cannot collapse.\r\n");
            continue;
        }

        // check if e->s is on a corner.
        bool onCorner = false;
        NPREdge::circulator begin;
        int degree = featureDegree(e->s(), begin);
        NPREdge::circulator ci = begin;
        NPREdge* prev_e = *begin;
        do {
            NPREdge* curr_e = *ci;
            if (curr_e == prev_e) continue;
            if (curr_e->count > 0 || (curr_e->twin() && curr_e->twin()->count > 0)) {
                Point3D v1 = prev_e->t()->p() - prev_e->s()->p();
                Point3D v2 = curr_e->t()->p() - curr_e->s()->p();
                if (v1.abs() > m_tol && v2.abs() > m_tol) {
                    v1.normalize();  v2.normalize();
                    real angle = acos(v1*v2)*180.0/M_PI;
                    if (angle < 150) onCorner = true;
                }
                prev_e = curr_e;
            }
        } while(++ci != begin && ci != 0 && !onCorner);
        if (onCorner && e->length() > m_tol) {
            Q.pop();
            e->qIterator = 0;
            /*
            e->s()->marked = true;
            e->t()->marked = true;
            CONS("removed because of angle\r\n");
            MB("hold hold");
            e->s()->marked = false;
            e->t()->marked = false;
            e->debug = true;
            */
            if (e->debug)
                CONS("Removed because on corner and long.\r\n");
            continue;
        } else if (onCorner) {
            Q.pop();
            e->qIterator = 0;
            int twin_feature_degree = featureDegree(e->t(), begin);
            if (e->twin() && e->twin()->qIterator == 0 && twin_feature_degree <= 2)
                e->twin()->qIterator = Q.push(e->length(), e->twin());
            if (e->debug)
                CONS("Removed because on corner.  Opposite might be inserted.\r\n");
            continue;
        }

        NPREdge* next = nextFeatureEdge(e);
        NPREdge* prev = prevFeatureEdge(e);
        ///*
        if (next == NULL || prev == NULL) {
            e->count = 0;
            if (e->twin()) e->twin()->count = 0;
            Q.pop();
            e->qIterator = 0;
            if (e->debug)
                CONS("Removed: next/prev does not exist.\r\n");
            continue;
            //CONS("qi guai le...\r\n");
            //e->s()->marked = true;
            //e->t()->marked = true;
            //MB("take a look");
        }
        //*/
        if (prev != NULL && next != NULL && e->length() > m_tol) {
            if ((prev->s()->p() - e->t()->p()).abs() >= 2*sampleDist) {
                if ((next->t()->p() - e->s()->p()).abs() < 2*sampleDist) {
                    Q.pop();
                    e->qIterator = 0;
                    if (e->twin() && e->twin()->qIterator == 0)
                        e->twin()->qIterator = Q.push(e->length(), e->twin());
                    if (e->debug)
                        CONS("Removed: merged will be too long, trying its twin.\r\n");
                    continue;
                } else {
                    Q.pop();
                    e->qIterator = 0;
                    if (e->debug)
                        CONS("Removed: merged will be too long.\r\n");
                    continue;
                }
            }
        }

        NPRVertex* t = e->t();

        ci = e->s()->edge_circulator();
        do {
            NPREdge* edge = *ci;
            if (edge->qIterator != 0) {
                Q.erase(edge->qIterator);
                edge->qIterator = 0;
            }
            if (edge->twin()->qIterator != 0) {
                Q.erase(edge->twin()->qIterator);
                edge->twin()->qIterator = 0;
            }
        } while (++ci != e->s()->edge_circulator() && ci != 0);
        ci = e->t()->edge_circulator();
        do {
            NPREdge* edge = *ci;
            if (edge->qIterator != 0) {
                Q.erase(edge->qIterator);
                edge->qIterator = 0;
            }
            if (edge->twin()->qIterator != 0) {
                Q.erase(edge->twin()->qIterator);
                edge->twin()->qIterator = 0;
            }
        } while (++ci != e->t()->edge_circulator() && ci != 0);

        /*
        e->s()->marked = true;
        e->t()->marked = true;
        CONS("Collapsing edge of length %f\r\n", e->length());
        MB("Test test");
        e->s()->marked = false;
        e->t()->marked = false;
        */

        if (prev != NULL) {
            m_normalMap[std::pair<NPRVertex*, NPRVertex*>(prev->s(), e->t())] =
                m_normalMap[std::pair<NPRVertex*, NPRVertex*>(prev->s(), prev->t())];
            m_normalMap[std::pair<NPRVertex*, NPRVertex*>(e->t(), prev->s())] =
                m_normalMap[std::pair<NPRVertex*, NPRVertex*>(prev->t(), prev->s())];
        }
        mesh.collapseEdge(e);

        ci = t->edge_circulator();
        do {
            NPREdge* edge = *ci;
            if (edge->twin()) {
                int c = max(edge->count, edge->twin()->count);
                edge->count = edge->twin()->count = c;
            }
            if (edge->length() < sampleDist &&
                (edge->count > 0 ||
                (edge->twin() && edge->twin()->count > 0))) {
                edge->qIterator = Q.push(edge->length(), edge);
            }
        } while (++ci != t->edge_circulator() && ci != 0);
    }
    //removeIsolatedVertices(mesh);
    //removeIsolatedAnchors(mesh);
    //restoreAnchors(mesh);
    //MB("mid point");
    return;

    List<NPREdge*> edge_list;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        if (edge->count <= 0) continue;
        if (edge->length() > 2*sampleDist) {
            edge_list.push_back(edge);
        }
    }
    foreach (ei, edge_list, List<NPREdge*>) {
        NPREdge* edge = *ei;
        NPRVertex* t = edge->t();
        NPRVertex* s = edge->s();
        if (!edge) continue;
        if (!edge->twin()) continue;
        if (edge->twin()->twin() != edge) continue;
        if (edge->length() <= 2*sampleDist) continue;
        if (edge->count <= 0) continue;
        int num_samples = int(edge->length()/sampleDist)-1;
        /*
        CONS("num_samples %d\r\n", num_samples);
        edge->s()->marked = true;
        edge->t()->marked = true;
        MB("hope it is fine...");
        edge->s()->marked = false;
        edge->t()->marked = false;
        */

        Point3D e_normal, o_normal;
        NormalMap::iterator itr =
            m_normalMap.find(std::pair<NPRVertex*, NPRVertex*>(s, t));
        if (itr != m_normalMap.end())
            e_normal = itr->second;
        else
            CONS("Warning: normal missing.\r\n");
        itr = m_normalMap.find(std::pair<NPRVertex*, NPRVertex*>(t, s));
        if (itr != m_normalMap.end())
            o_normal = itr->second;
        else
            CONS("Warning: normal missing.\r\n");

        List<Point3D> p_list;
        for (int i=0; i<num_samples; i++) {
            Point3D p = edge->t()->p()*double(i+1)/double(num_samples+1)
                + edge->s()->p()*(1-double(i+1)/double(num_samples+1));
            p_list.push_back(p);
        }
        foreach (pi, p_list, List<Point3D>){
            Point3D p = *pi;
            NPRVertex* v = splitEdge(mesh, edge, p);
            NPREdge* first_half = s->getEdgeTo(v);
            NPREdge* second_half = v->getEdgeTo(t);
            ASSERT(first_half != NULL);
            ASSERT(second_half != NULL);
            ASSERT(first_half->twin());
            ASSERT(second_half->twin());
            first_half->count = second_half->count
                              = first_half->twin()->count
                              = second_half->twin()->count
                              = m_drawerB.getMaxTol();
            // Register normals
            m_normalMap[std::pair<NPRVertex*, NPRVertex*>(s,v)] = e_normal;
            m_normalMap[std::pair<NPRVertex*, NPRVertex*>(v,t)] = e_normal;
            m_normalMap[std::pair<NPRVertex*, NPRVertex*>(v,s)] = o_normal;
            m_normalMap[std::pair<NPRVertex*, NPRVertex*>(t,v)] = o_normal;
            edge = second_half;
            s = v;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

real CNPRAlg::getAveFeatureEdgeDist(const NPRMesh& mesh) const
{
    real total_len = 0.0;
    int count = 0;
    foreach_const(ei, mesh.edges(), NPRMesh::Edges) {
        const NPREdge* e = &*ei;
        if (e->count > 0) {
            total_len += e->length();
            count++;
        }
    }
    if (count == 0) return 0.0;
    else return total_len / count;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

real CNPRAlg::getAveStrokeDist() const
{
    real total_len = 0.0;
    int count = 0;
    foreach_const (si, m_strokes, Strokes) {
        const Stroke* stroke = &*si;
        total_len += stroke->length();
        count += (stroke->size()-1);
    }
    if (count == 0) return 0.0;
    else return total_len / count;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Join connect strokes
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::joinPieces(NPRMesh& mesh)
{
    Array<int> end_array(mesh.nVertices(), -1);
    int count = 0;
    Array<Stroke*> stroke_array(m_strokes.size(), NULL);
    foreach (si, m_strokes, Strokes) {
        int stroke_index = count;
        Stroke* stroke = &*si;
        ASSERT(!stroke->empty());
        stroke_array[count] = stroke;
        SVertex* first = &stroke->front();
        SVertex* last  = &stroke->back();
        if (end_array[first->vertex->m_index] != -1) {
            Stroke* other = stroke_array[end_array[first->vertex->m_index]];
            ASSERT(!other->empty());
            end_array[other->front().vertex->m_index] = -1;
            end_array[other->back().vertex->m_index] = -1;
            if (other->back().vertex == first->vertex) {
                other->reverse();
            }
            ASSERT(other->front().vertex == first->vertex);
            stroke->pop_front(); // remove the repeated element
            foreach (svi, (*other), Stroke) {
                stroke->push_front(*svi);
            }
            other->clear();
            end_array[first->vertex->m_index] = -1;
            first = &stroke->front();
        }
        if (end_array[last->vertex->m_index] != -1) {
            Stroke* other = stroke_array[end_array[last->vertex->m_index]];
            ASSERT(!other->empty());
            end_array[other->front().vertex->m_index] = -1;
            end_array[other->back().vertex->m_index] = -1;
            if (other->back().vertex == last->vertex) {
                other->reverse();
            }
            ASSERT(other->front().vertex == last->vertex);
            stroke->pop_back();
            foreach (svi, (*other), Stroke) {
                stroke->push_back(*svi);
            }
            other->clear();
            end_array[last->vertex->m_index] = -1;
            last = &stroke->back();
        }
        if (first != last) {
            end_array[first->vertex->m_index] = stroke_index;
            end_array[last->vertex->m_index] = stroke_index;
        } else {
            end_array[first->vertex->m_index] = -1;
            end_array[last->vertex->m_index] = -1;
            stroke->circular = true;
        }
        count++;
    }

    foreach (si, m_strokes, Strokes) {
        Stroke* stroke = &*si;
        if (stroke->empty()) {
            Strokes::iterator prev = si;
            prev--;
            m_strokes.erase(si);
            si = prev;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Return true if the 3-loop cannot be removed
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::remove_3_loop(NPRMesh& mesh, NPRVertex* v)
{
    if (v->nNeighbors() != 3) return false;
    NPREdge::circulator ci = v->edge_circulator();
    do {
        NPREdge* edge = *ci;
        if (mesh.collapsedCheck(edge)) {
            mesh.collapseEdge(edge);
            return false;
        }
    } while (++ci != v->edge_circulator() && ci != 0);
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::smoothCheck(Point3D newPosV1, Point3D newPosV2, Point3D newPosV3, 
						  Point3D normalV1, Point3D normalV2, Point3D normalV3, 
						  real angle_tol)
{
    Point3D nFaceNormal = triangleNormal(newPosV1, newPosV2, newPosV3);
    nFaceNormal.normalize();
    real maxAngle = max(
        acos(valid_cos(nFaceNormal*normalV1)),
        acos(valid_cos(nFaceNormal*normalV2)),
        acos(valid_cos(nFaceNormal*normalV3)));
    maxAngle = maxAngle*180.0/M_PI;

	 return (maxAngle < angle_tol);
}



/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::distCheck(NPRFace* face, real angle_tol)
{
    real maxAngle = max(
        acos(valid_cos(face->v0()->normal * face->v1()->normal)),
        acos(valid_cos(face->v0()->normal * face->v2()->normal)),
        acos(valid_cos(face->v1()->normal * face->v2()->normal)));
    maxAngle = maxAngle*180.0/M_PI;
    return (maxAngle < angle_tol);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Cut long edge in half
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::cutLongEdges(NPRMesh& mesh, real dist_tol)
{
    EdgeQueue Q;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        edge->qIterator = 0;
        if (edge->length() > dist_tol) {
            edge->qIterator = Q.push(-1*edge->length(), edge);
            if (edge->debug) {
                CONS("Inserted\r\n");
            }
        }
    }

    while (!Q.empty()) {
        NPREdge* e = Q.top_value();
        //e->qIterator = 0;
        //Q.pop();

        NPREdge* temp = e;
        do {
            if (temp->qIterator != 0) {
                Q.erase(temp->qIterator);
                temp->qIterator = 0;
            }
            if (temp->twin() && temp->twin()->qIterator != 0) {
                Q.erase(temp->twin()->qIterator);
                temp->twin()->qIterator = 0;
            }
            temp = temp->nextInFace();
        } while (temp != e);
        if (e->twin()) {
            temp = e->twin();
            do {
                if (temp->qIterator != 0) {
                    Q.erase(temp->qIterator);
                    temp->qIterator = 0;
                }
                if (temp->twin() && temp->twin()->qIterator != 0) {
                    Q.erase(temp->twin()->qIterator);
                    temp->twin()->qIterator = 0;
                }
                temp = temp->nextInFace();
            } while (temp != e->twin());
        }

        Point3D p = (e->s()->p() + e->t()->p()) * 0.5;
        NPRVertex* v = splitEdge(mesh, e, p);

        NPREdge::circulator ci = v->edge_circulator();
        do {
            NPREdge* edge = *ci;
            do {
                if (edge->length() > dist_tol && edge->qIterator == 0) {
                    edge->qIterator = Q.push(-1*edge->length(), edge);
                }
                edge = edge->nextInFace();
            } while (edge != *ci);

            edge->face()->color() = Color3d(0.0, 1.0, 0.0);
        } while (++ci != v->edge_circulator() && ci != 0);
    }
    return;

    /*
    List<NPREdge*> toCut;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        edge->visited = false;
    }
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        if (edge->visited) continue;

        edge->visited = true;
        if (edge->twin()) edge->twin()->visited = true;
        if (edge->length() > dist_tol) {
            toCut.push_back(edge);
        }
    }
    foreach (ei, toCut, List<NPREdge*>) {
        NPREdge* edge = *ei;
        List<NPREdge*>::iterator next = ei.next();
        if (!edge->twin() || edge->twin()->twin() != edge) {
            continue;
        }
        Point3D p = (edge->s()->p() + edge->t()->p()) * 0.5;
        edge->count = 180;
        (*next)->count = 1;
        MB("pause.\r\n");
        edge->count = 0;
        (*next)->count = 0;
        NPRVertex* v = splitEdge(mesh, edge, p);
        (*next)->count = 1;
        MB("look");
        (*next)->count = 0;
        ei = next.prev();

        NPREdge::circulator ci = v->edge_circulator();
        do {
            (*ci)->face()->color() = Color3d(0.0, 1.0, 0.0);
        } while (++ci != v->edge_circulator() && ci != 0);
    }
    */
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::filterMisMappings_old(NPRMesh& mesh)
{
    List<NPRVertex*> wrong_mappings;
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = &*vi;
        NPRFace* mapped_face = (NPRFace*)v->mapped_face;
        if (mapped_face == NULL) continue;
        Point3D mapped_normal = mapped_face->m_normal;

        bool wrong = false;
        NPREdge::circulator ci = v->edge_circulator();
        do {
            NPREdge* e = *ci;
            if (e->s()->nearestPoint.abs() < epsilon() ||
                e->t()->nearestPoint.abs() < epsilon() ||
                e->nextInFace()->t()->nearestPoint.abs() < epsilon()) {
                    continue;
            }
            real old_area = area(
                e->s()->p(),
                e->t()->p(),
                e->nextInFace()->t()->p());
            real new_area = area(
                e->s()->nearestPoint,
                e->t()->nearestPoint,
                e->nextInFace()->t()->nearestPoint);
            if (old_area < epsilon() || new_area < epsilon()) {
                if (e->debug) {
                    CONS("Area is too small.\r\n");
                }
                continue;
            }
            if (old_area / new_area > 3) {
                if (e->debug) {
                    CONS("Area ratio is too big.\r\n");
                }
                continue;
            }
            Point3D old_normal = triangleNormal(
                e->s()->p(),
                e->t()->p(),
                e->nextInFace()->t()->p());
            Point3D new_normal = triangleNormal(
                e->s()->nearestPoint,
                e->t()->nearestPoint,
                e->nextInFace()->t()->nearestPoint);

            if (old_normal.abs() < epsilon() || new_normal.abs() < epsilon()) {
                if (e->debug) {
                    CONS("Normal not well defined.\r\n");
                }
                continue;
            }
            old_normal.normalize();
            new_normal.normalize();
            if (new_normal*mapped_normal < -epsilon())
                new_normal = new_normal * -1;
            real angle = acos(old_normal * new_normal)*180.0/M_PI;
            if (e->debug) {
                CONS("Angle is %f\r\n", angle);
            }
            if (angle > 100) {
                wrong = true;
                break;
            }
            /*
            if (!e->twin()) continue;
            if (e->s()->nearestPoint.abs() < epsilon() ||
                e->t()->nearestPoint.abs() < epsilon() ||
                e->nextInFace()->t()->nearestPoint.abs() < epsilon() ||
                e->twin()->nextInFace()->t()->nearestPoint.abs() < epsilon()) {
                    continue;
            }

            real oarea1 = area(
                e->s()->p(),
                e->t()->p(),
                e->nextInFace()->t()->p());
            real oarea2 = area(
                e->twin()->s()->p(),
                e->twin()->t()->p(),
                e->twin()->nextInFace()->t()->p());
            real area1 = area(
                e->s()->nearestPoint,
                e->t()->nearestPoint,
                e->nextInFace()->t()->nearestPoint);
            real area2 = area(
                e->twin()->s()->nearestPoint,
                e->twin()->t()->nearestPoint,
                e->twin()->nextInFace()->t()->nearestPoint);

            if (oarea1 < epsilon() || oarea2 < epsilon()) {
                continue;
            }
            if (area1 < epsilon() || area2 < epsilon()) {
                continue;
            }

            Point3D normal1 = triangleNormal(
                e->s()->nearestPoint,
                e->t()->nearestPoint,
                e->nextInFace()->t()->nearestPoint);
            Point3D normal2 = triangleNormal(
                e->twin()->s()->nearestPoint,
                e->twin()->t()->nearestPoint,
                e->twin()->nextInFace()->t()->nearestPoint);

            if (normal1.abs() < epsilon() || normal2.abs() < epsilon()) {
                continue;
            }

            normal1.normalize();
            normal2.normalize();
            real angle = acos(normal1*normal2)*180.0/M_PI;
            if (angle > 150) {
                wrong = true;
                break;
            }
            */
        } while (++ci != v->edge_circulator() && ci != 0);
        if (wrong) {
            wrong_mappings.push_back(v);
            v->marked = true;
        }
    }

    MB("Time to think...");
    foreach (vi, wrong_mappings, List<NPRVertex*>) {
        NPRVertex* v = *vi;
        v->nearestPoint *= 0;
        v->marked = false;
    }

    foreach (vi, wrong_mappings, List<NPRVertex*>) {
        NPRVertex* v = *vi;
        if ((v->p() - v->nearestPoint).abs() < m_tol) continue;
        bool converged = true;
        NPREdge::circulator ci = v->edge_circulator();
        Point3D center;
        int neighbors = 0;
        do {
            NPRVertex* t = (*ci)->t();
            if (t->nearestPoint.abs() < epsilon()) continue;
            if ((t->nearestPoint - t->p()).abs() > m_tol) {
                converged = false;
                break;
            }
            center += t->nearestPoint;
            neighbors++;
        } while (++ci != v->edge_circulator() && ci != 0);

        if (converged && neighbors > 0) {
            v->nearestPoint = center/neighbors;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::filterMisMappings(NPRMesh &mesh)
{
    mesh.computeFaceAreas(0);
    mesh.computeFaceNormals(0);
    mesh.computeVertexNormals(0);

    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        NPRVertex* v = &*vi;
        if (v->mapped_face == NULL) continue;
        ASSERT(v->nearestPoint.abs() > epsilon());
        Point3D mapping = v->nearestPoint - v->p();
        if (mapping.abs() < m_tol) continue;
        mapping.normalize();
        NPREdge::circulator ci = v->edge_circulator();
        do {
            Point3D normal = (*ci)->face()->m_normal;
            real angle = acos(normal * mapping)*180/M_PI;
            if (angle < 89.5) {
                v->nearestPoint *= 0;
                break;
            }
        } while (++ci != v->edge_circulator() && ci != 0);
        /*
        Point3D normal = v->normal;
        Point3D mapping = v->nearestPoint - v->p();
        if (mapping.abs() < m_tol) continue;
        mapping.normalize();
        real angle = acos(normal * mapping)*180/M_PI;
        if (angle < 120) {
            // Mapping to a point out of voxel hull, probably wrong.
            v->nearestPoint *= 0;
        }
        */
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::isEdgeFlippable( NPREdge* e, NPRMesh& refMesh, double& diffDistance )
{
	if( !e || !e->twin() || !e->nextInFace() || !e->twin()->nextInFace() ) return false;
	
	NPRVertex*  s = e->s(),
			 *  t = e->t(),
			 *	v0 = e->nextInFace()->t(),
			 *	v1 = e->twin()->nextInFace()->t();


	// there shouldn't already be an edge there. will make mesh non-manifold
    if( v0->getEdgeTo(v1) != 0 ) {
        if (e->debug)
            CONS("Unable to flip due to topology...\r\n");
		return false;
    }
	
	// to prevent degenerate triangles
	Point3D v0_s = (v0->p() - s->p()).unit(),
			v1_s = (v1->p() - s->p()).unit(),
			v0_t = (v0->p() - t->p()).unit(),
			v1_t = (v1->p() - t->p()).unit();
	double	angle1 = acos(valid_cos( dot( v0_s, v1_s ) ) )*180.0/M_PI,
			angle2 = acos(valid_cos( dot( v0_t, v1_t ) ) )*180.0/M_PI;
	if( angle1 > 150 || angle2 > 150 || ( angle1 + angle2 ) > 320 )
	{
		//e->count = 10;
        if (e->debug)
            CONS("Unable to flip due to degeneracy...\r\n");
		return false;
	}


	// if the triangle flips
	Point3D normal1 = cross(v1_s, v0_s).unit(),
			normal2 = cross(v0_t, v1_t).unit();
	double angle = acos(valid_cos( dot(normal1,normal2) ) )*180.0/M_PI;
	if( angle > 165 )
	{
		//e->count = 10;
        if (e->debug)
            CONS("Flipping will cause flipped triangle...\r\n");
		return false;
	}
	
	// already projected onto the original mesh 
	// if there is no mapped face, return 
	if( !(s->mapped_face != NULL && t->mapped_face != NULL && v0->mapped_face != NULL && v1->mapped_face != NULL ) )
	{
		//e->count = 10;
        if (e->debug)
            CONS("Unable to flip because end points are not mapped...\r\n");
		return false;
	}
	

	
	// only perform this flip operation on points that lie on the original surface
	double	sDist = (s->nearestPoint - s->p()).abs(),
		tDist = (t->nearestPoint - t->p()).abs(),
		v0Dist = (v0->nearestPoint - v0->p()).abs(),
		v1Dist = (v1->nearestPoint - v1->p()).abs();
	double closeThres = 0.05;
	if( !(sDist < closeThres && tDist < closeThres && v0Dist < closeThres && v1Dist < closeThres ) )
	{
		e->count = 10;
        if (e->debug)
            CONS("Unable to flip because local neighborhood does not lie on the original mesh...\r\n");
		return false;
	}
	
	// position of mid points of edges
	Point3D pMid = ( e->s()->p() + e->t()->p() )/2.0, 
		pFlipMid = ( v0->p() + v1->p() )/2.0;
	Point3D	pMidProj, pFlipMidProj;

    NPRFace* f1 = getNearestPoint(pMid, refMesh, pMidProj);
    NPRFace* f2 = getNearestPoint(pFlipMid, refMesh, pFlipMidProj);
    if( f1 == NULL ) {
        if (e->debug)
            CONS("Unable to flip because mid point cannot be mapped...\r\n");
        return false;
    }
    if( f2 == NULL ) {
        if (e->debug)
            CONS("Unable to flip because flipped mid point cannot be mapped...\r\n");
        return false;
    }
    /*
    if (f1 != f2) {
        if (e->debug)
            CONS("Unable to flip because old and new mid points mapped to difference faces...\r\n");
        return false;
    }
    */

	double	eMapDistance = ( pMidProj - pMid ).abs(),
		eFlipMapDistance = ( pFlipMidProj - pFlipMid ).abs();

	diffDistance = eFlipMapDistance - eMapDistance;
	bool result = (diffDistance < -0.001/*-epsilon()*/);

	if( result )
	{
		if( !neighborhoodNormalTest( normal1, normal2, e ) )
		{
			e->count = 180;	
            if (e->debug)
                CONS("Unable to flip because normal test failed...\r\n");
			return false;
		}
    } else {
        if (e->debug)
            CONS("Unable to flip because it does not improve the midpoint distance...\r\n");
    }
		
	return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

bool CNPRAlg::neighborhoodNormalTest( Point3D newNormal1, Point3D newNormal2, NPREdge* e )
{
	if( !e->twin() )
		return false;

	Point3D avgNeighNormal1(0.0, 0.0, 0.0);
	Point3D avgNeighNormal2(0.0, 0.0, 0.0);
    Point3D avgNeighNormal3(0.0, 0.0, 0.0);
    Point3D avgNeighNormal4(0.0, 0.0, 0.0);
    Point3D oldNormal1 = e->face()->m_normal;
    Point3D oldNormal2 = e->twin()->face()->m_normal;
    double aveAngle1 = 0.0, aveAngle2 = 0.0;
    double weight1 = 0.0, weight2 = 0.0;
    /*
	NPREdge* te;

	te = e->nextInFace();
	while( te != e )
	{
		NPREdge* twinEdge = te->twin();
		NPRFace* face = twinEdge->face();
		avgNeighNormal += face->m_normal * face->m_area;
	
		te = te->nextInFace();
	}

	te = e->twin()->nextInFace();
	while( te != e->twin() )
	{
		NPREdge* twinEdge = te->twin();
		NPRFace* face = twinEdge->face();
		avgNeighNormal += face->m_normal * face->m_area;
	
		te = te->nextInFace();
	}
    */
#if 1
    NPREdge* edge = e->nextOfS();
    while (edge != e->twin()->nextInFace()) {
        avgNeighNormal1 += edge->face()->m_normal * edge->face()->m_area;
        double angle = acos(valid_cos(edge->face()->m_normal * newNormal1))*180.0/M_PI;
        double weight = edge->face()->m_area;
        aveAngle1 += angle*weight;
        weight1 += weight;
        edge = edge->nextOfS();
    }
    avgNeighNormal1.normalize();
    aveAngle1 /= weight1;

    edge = e->twin()->nextOfS();
    while (edge != e->nextInFace()) {
        avgNeighNormal2 += edge->face()->m_normal * edge->face()->m_area;
        double angle = acos(valid_cos(edge->face()->m_normal * newNormal2))*180.0/M_PI;
        double weight = edge->face()->m_area;
        aveAngle2 += angle*weight;
        weight2 += weight;
        edge = edge->nextOfS();
    }
    avgNeighNormal2.normalize();
    aveAngle2 /= weight2;
#else
    avgNeighNormal1 += e->nextInFace()->twin()->face()->m_normal
        * e->nextInFace()->twin()->face()->m_area;
    avgNeighNormal1 += e->twin()->prevInFace()->twin()->face()->m_normal
        * e->twin()->prevInFace()->twin()->face()->m_area;
    avgNeighNormal1.normalize();
    avgNeighNormal2 += e->prevInFace()->twin()->face()->m_normal
        * e->prevInFace()->twin()->face()->m_area;
    avgNeighNormal2 += e->twin()->nextInFace()->twin()->face()->m_normal
        * e->twin()->nextInFace()->twin()->face()->m_area;
    avgNeighNormal2.normalize();
    avgNeighNormal3 += e->nextInFace()->twin()->face()->m_normal
        * e->nextInFace()->twin()->face()->m_area;
    avgNeighNormal3 += e->prevInFace()->twin()->face()->m_normal
        * e->prevInFace()->twin()->face()->m_area;
    avgNeighNormal3.normalize();
    avgNeighNormal4 += e->twin()->nextInFace()->twin()->face()->m_normal
        * e->twin()->nextInFace()->twin()->face()->m_area;
    avgNeighNormal4 += e->twin()->prevInFace()->twin()->face()->m_normal
        * e->twin()->prevInFace()->twin()->face()->m_area;
    avgNeighNormal4.normalize();
#endif

	//avgNeighNormal.normalize();
	//CONS("AvgNormal %f %f %f \r\n", avgNeighNormal.x(), avgNeighNormal.y(), avgNeighNormal.z() );

	double	angle1 = acos(valid_cos( dot( newNormal1, avgNeighNormal1 ) ) )*180.0/M_PI,
			angle2 = acos(valid_cos( dot( newNormal2, avgNeighNormal2 ) ) )*180.0/M_PI;
    //double  angle3 = acos(valid_cos( dot( oldNormal1, avgNeighNormal3 ) ) )*180.0/M_PI,
	//        angle4 = acos(valid_cos( dot( oldNormal2, avgNeighNormal4 ) ) )*180.0/M_PI;
    if (e->debug) {
    //    CONS("Old angles: %f %f\r\n", angle3, angle4);
        CONS("New angles: %f %f\r\n", angle1, angle2);
        CONS("Ave angles: %f %f\r\n", aveAngle1, aveAngle2);
    }
#if 1
	if( (aveAngle1 > m_angleTol && aveAngle2 > m_angleTol) || aveAngle1 > 120 || aveAngle2 > 120 )
	{
        //if (e->debug)
		//    CONS("Angles  %f %f\r\n", angle1, angle2 );
		return false;
	}
#else
    if ((angle1+angle2) > (angle3+angle4)) {
        if (e->debug)
            CONS(" %f < %f\r\n", (angle1+angle2), (angle3+angle4));
        return false;
    }
#endif
	else
		return true;
}


/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::autoDetectVirtualEdges(NPRMesh& mesh)
{
	CONS("Virtual edge threshold %f \r\n", m_virtualAngleTol );

    foreach (fi, mesh.faces(), NPRMesh::Faces) {
        NPREdge* edge;
        NPREdge::circulator begin;
        foreach (ei, fi->edges(), List<NPREdge*>) {
            edge = *ei;
            int degree = edge->s()->degree();
            //int degree = featureDegree(edge->s(), begin);
            if (degree > 2) break;
        }

        NPREdge* temp = edge;
        List<NPREdge*> segment;
        real totalAngle = 0.0;
        do {
            ASSERT(temp->twin());
            Point3D n1 = temp->normal;
            Point3D n2 = temp->twin()->normal;
            n1.normalize(); n2.normalize();
            double angle = acos(n1*n2) * 180.0 / M_PI;
            if (isnan(angle)) {
                //temp->s()->marked = true;
                //temp->t()->marked = true;
            } else {
                segment.push_back(temp);
                totalAngle += angle;
            }

            int nextDegree = temp->t()->degree();
            //int nextDegree = featureDegree(temp->t(), begin);
            if (nextDegree >2) {
                real aveAngle = totalAngle / segment.size();
                //CONS("ave angle: %f\r\n", aveAngle);
                if (aveAngle < m_virtualAngleTol) {
                    foreach (ei, segment, List<NPREdge*>) {
                        (*ei)->count = 0;
                        (*ei)->twin()->count = 0;
                    }
                }
                totalAngle = 0.0;
                segment.clear();
            }
            temp = temp->nextInFace();
        } while (temp != edge);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::scaleMappings(NPRMesh& mesh)
{
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        if (vi->mapped_face == NULL) continue;
        Point3D mapped = vi->nearestPoint;
        Point3D p = vi->p();
        if (mapped.abs() < epsilon()) continue;
        Point3D scaled = p + (mapped-p)*0.9;
        vi->nearestPoint = scaled;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::correctSelfIntersection(NPRFace* f1, NPRFace* f2)
{
    //if (f1->m_normal * f1->centroid() > 0) {
    if (f1->m_visRatio > f2->m_visRatio) {
        // swap f1 and f2
        NPRFace* temp = f1;
        f1 = f2;
        f2 = temp;
    }

    // alwayse move f1
    Point3D n2 = f2->m_normal;
    f1->color() = Color3d(1.0, 0.0, 0.0);
    f2->color() = Color3d(0.0, 0.0, 1.0);
    real dist = f2->v0()->p() * n2;
    NPREdge* edge = f1->e0();
    do {
        real dist2 = edge->s()->p() * n2;
        if (false) {
            f1->v0()->marked = true;
            f1->v1()->marked = true;
            f1->v2()->marked = true;
            f2->v0()->marked = true;
            f2->v1()->marked = true;
            f2->v2()->marked = true;
            MB("test test");
            f1->v0()->marked = false;
            f1->v1()->marked = false;
            f1->v2()->marked = false;
            f2->v0()->marked = false;
            f2->v1()->marked = false;
            f2->v2()->marked = false;
        }
        if (dist2 > dist && dist2 < dist + 0.5) {
            edge->s()->p() += n2 * -1.0 * (dist2-dist + m_tol);
        }

        edge = edge->nextInFace();
    } while (edge != f1->e0());
}

/////////////////////////////////////////////////////////////////////////////////////////
// Break all 3 loop with one or more edges < tol
/////////////////////////////////////////////////////////////////////////////////////////

int CNPRAlg::break3Loops(NPRMesh& mesh, real tol)
{
    EdgeQueue Q;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* edge = &*ei;
        edge->qIterator = 0;

        if (edge->length() < tol && !mesh.collapsedCheck(edge)) {
            edge->qIterator = Q.push(edge->length(), edge);
            if (edge->debug) {
                CONS("Inserted\r\n");
            }
        }
    }

    while (!Q.empty()) {
        NPREdge* edge = Q.top_value();
        NPREdge* edge2 = NULL;
        NPREdge* edge3 = NULL;

        // Find all edges of the 3-loop
        NPRVertex* s = edge->s();
        NPRVertex* t = edge->t();
        NPRVertex* p = NULL;
        NPREdge::circulator si = s->edge_circulator();
        do {
            NPREdge* s_edge = *si;
            NPREdge::circulator ti = t->edge_circulator();
            do {
                NPREdge* t_edge = *ti;
                if (s_edge->t() == t_edge->t() &&
                    mesh.getFace(s, t, s_edge->t()) == 0) {
                        // Detected a 3-loop
                        // (t)<-edge---(s)
                        //  \           ^
                        //   \         /
                        //  edge_2   edge_3
                        //     \     /
                        //      V   /
                        //       (p)
                        ASSERT(s_edge->twin());
                        edge3 = s_edge->twin();
                        edge2 = t_edge;
                        p = t_edge->t();
                }
            } while (++ti != t->edge_circulator() && ti != 0 &&
                edge2 == NULL && edge3 == NULL);
        } while (++si != s->edge_circulator() && si != 0 &&
            edge2 == NULL && edge3 == NULL);

        ASSERT(edge2 != NULL);
        ASSERT(edge3 != NULL);
        ASSERT(p     != NULL);

        List<NPREdge*> involvedEdges;
        involvedEdges.push_back(edge);
        involvedEdges.push_back(edge2);
        involvedEdges.push_back(edge3);
        involvedEdges.push_back(edge->twin());
        involvedEdges.push_back(edge2->twin());
        involvedEdges.push_back(edge3->twin());

        foreach (ei, involvedEdges, List<NPREdge*>) {
            removeFromQ(Q, *ei);
        }

        cut(mesh, edge, edge2, edge3);

        foreach (ei, involvedEdges, List<NPREdge*>) {
            NPREdge* e = *ei;
            if (!e || !e->twin() || e->twin()->twin() != e)
                continue;

            if (e->length() < tol && !mesh.collapsedCheck(e)) {
                e->qIterator = Q.push(e->length(), e);
            }
        }
    }
    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::eraseNeighborsFromQ(EdgeQueue& Q, NPRVertex* v)
{
    ASSERT(v != NULL);
    NPREdge::circulator ci = v->edge_circulator();
    do {
        NPREdge* edge = *ci;
        removeFromQ(Q, edge);
        if (edge->twin())
            removeFromQ(Q, edge->twin());
    } while (++ci != v->edge_circulator() && ci != 0);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Cut 3-loop formed by e0, e1, e2
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::cut(NPRMesh &mesh, NPREdge *e0, NPREdge *e1, NPREdge *e2)
{
    ASSERT(e0->t() == e1->s());
    ASSERT(e1->t() == e2->s());
    ASSERT(e2->t() == e0->s());
    ASSERT(e0->face() != e1->face()
        && e1->face() != e2->face()
        && e2->face() != e0->face());

    NPRVertex* v0 = e0->s();
    NPRVertex* v1 = e1->s();
    NPRVertex* v2 = e2->s();
    NPRVertex* nv0 = mesh.addVertex(v0->p());
    NPRVertex* nv1 = mesh.addVertex(v1->p());
    NPRVertex* nv2 = mesh.addVertex(v2->p());
    e0->detachTwin();
    e1->detachTwin();
    e2->detachTwin();

    v0->moveConnectedRegion(e0, nv0);
    v1->moveConnectedRegion(e1, nv1);
    v2->moveConnectedRegion(e2, nv2);
    mesh.addFace(v0,  v1,  v2);
    mesh.addFace(nv2, nv1, nv0);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::calculateCreaseAngles(NPRMesh& mesh, real creaseThreshold, int pid )
{
    const real threshold = (creaseThreshold) * M_PI / 180.0  - epsilon();
	//const real eps = 1e-1;
    
	foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPRMesh::Edge* e = &(*ei);
        NPRMesh::Edge* twin = e->twin();
		if (twin == 0) 
		{
			e->eCreaseAngle = 180.0;
			e->eCrease = true;
			continue;
		}

        NPRMesh::Face* f0 = e->face();
        NPRMesh::Face* f1 = twin->face();
        Point3D n0 = triangleFaceNormal(f0, pid);
        Point3D n1 = triangleFaceNormal(f1, pid);

		//degeneracy check //////////
		if ( n0.abs() < epsilon() || n1.abs() < epsilon() ) continue;

        real a = angle(n0, n1); // unitizes
		e->eCreaseAngle = a * 180.0 / M_PI;
        e->eCrease = (a >= threshold);
    }
    mesh.geometrySensor().modify();
    return;
}


NPREdge* CNPRAlg::getOppositeEdge( NPREdge* e, NPRMesh& mesh )
{
	Point3D p1 = e->s()->p(),
			p2 = e->t()->p();

	foreach (ei, mesh.edges(), NPRMesh::Edges)
	{
		NPREdge* newe = &*ei;

		if( newe->twin() == 0 && newe != e )
		{
			Point3D p3 = newe->s()->p(),
					p4 = newe->t()->p();

			if( ( (p1-p3 ).abs() < 0.001 && (p2-p4).abs() < 0.001 ) ||
				( (p1-p4 ).abs() < 0.001 && (p2-p3).abs() < 0.001 ) )
				return newe;
		}
	}
	CONS("No opposite edge found !!! \r\n MESH DOESNOT HAVE MULTIPLE COMPONENTS \r\n "); 
	//ASSERT( false );
	return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::calculateCreaseAnglesForGeometryEdges(NPRMesh& mesh, real creaseThreshold, int pid )
{
	CONS("Calculating crease angles for geometry edges \r\n");
    const real threshold = (creaseThreshold) * M_PI / 180.0  - epsilon();
	 
	foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPRMesh::Edge* e = &(*ei);
        NPRMesh::Edge* twin = e->twin();
        
		if (twin == 0)
		{
			twin = getOppositeEdge(e, mesh);

			if( twin == NULL )
				return;

			NPRMesh::Face* f0 = e->face();
			NPRMesh::Face* f1 = twin->face();
			Point3D n0 = triangleFaceNormal(f0, pid);
			Point3D n1 = triangleFaceNormal(f1, pid);

			n0.normalize();
			n1.normalize();

			//degeneracy check //////////
			if ( n0.abs() < epsilon() || n1.abs() < epsilon() ) continue;

			real a = angle(n0, n1); // unitizes
			e->eCreaseAngle = a * 180.0 / M_PI;
			e->eCrease = (a >= threshold);
			
			twin->eCreaseAngle = a * 180.0 / M_PI;
			twin->eCrease = (a >= threshold);
			
			//CONS("Crease angle %f \r\n", e->eCreaseAngle );
		}
    }
	CONS("Calculated \r\n");

    mesh.geometrySensor().modify();
    return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::smoothFeatureSegmentNormal(NPRMesh &mesh, real radius)
{
    int min_tol = m_drawerB.getMinTol();
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        ASSERT(ei->twin());
        ei->visited = false;
    }

    std::map<NPREdge*, Point3D> normal_map;

    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* e = &*ei;
        if (e->count <= min_tol && e->twin()->count <= min_tol) continue;
        if (e->visited) continue;
        e->visited = true;

        Point3D normal = e->normal;
        NPREdge::circulator dummy;
        // One direction
        NPREdge* tmp = e;
        real dist = e->length()/2.0;
        while (dist < radius) {
            if (featureDegree(tmp->s(), dummy) != 2) break;
            tmp = prevFeatureEdge(tmp);
            dist += tmp->length();
            real weight = exp(-dist/radius);
            normal += tmp->normal * weight;
        }
        // The other direction
        tmp = e;
        dist = e->length()/2.0;
        while (dist < radius) {
            if (featureDegree(tmp->t(), dummy) != 2) break;
            tmp = nextFeatureEdge(tmp);
            dist += tmp->length();
            real weight = exp(-dist/radius);
            normal += tmp->normal * weight;
        }

        normal.normalize();
        normal_map[e] = normal;
    }

    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        NPREdge* e = &*ei;
        std::map<NPREdge*, Point3D>::iterator i = normal_map.find(e);
        if (i != normal_map.end())
            e->normal = normal_map[e];
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::markSampledVertex(NPRMesh& mesh, real dist)
{
    int min_tol = m_drawerB.getMinTol();
    NPREdge::circulator dummy;
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        ei->visited = false;
    }
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        vi->marked = false;
    }
    if (dist < epsilon()) return;

    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        if (ei->visited) continue;
        if (ei->count <= min_tol && (!ei->twin() || ei->twin()->count <= min_tol)) continue;
        int degree = featureDegree(ei->s(), dummy);
        if (degree <= 2) continue;
        NPREdge* e = &*ei;
        std::vector<NPRVertex*> vertices;
        real len = 0.0;
        //real angle_diff = 0.0;
        NPREdge* prev = e;
        do {
            vertices.push_back(e->s());
            e->visited = true;
            e->twin()->visited = true;
            len += e->length();
            //real angle1 = acos(valid_cos(e->normal * prev->normal))*180.0/M_PI;
            //real angle2 = acos(valid_cos(e->twin()->normal * prev->twin()->normal))*180.0/M_PI;
            //angle_diff += (angle1 + angle2)*0.5;
            prev = e;
            e = nextFeatureEdge(e);
            if (e != NULL)
                degree = featureDegree(e->s(), dummy);
        } while (degree == 2 && e != NULL);
        if (e != NULL)
            vertices.push_back(e->s());
        if (ei->isProp(EP_STRAIGHT) || ei->isProp(EP_CIRCLE_SEGMENT)) {
            vertices[vertices.size()/2]->marked = true;
        } else if (ei->isProp(EP_CIRCLE)) {
            // Full circle should  have no T-junctions.
        } else {
            subSample(vertices, 0, vertices.size()-1, len, dist);
        }
        /*
        angle_diff /= (vertices.size()-2);
        if (angle_diff < 1) {
            // Straight line.
            vertices[vertices.size()/2]->marked = true;
        } else {
            // General edge.
            subSample(vertices, 0, vertices.size()-1, len, dist);
        }
        */
    }

    // Handle feature loops without T-junction
    foreach (ei, mesh.edges(), NPRMesh::Edges) {
        if (ei->visited) continue;
        if (ei->count <= min_tol && (!ei->twin() || ei->twin()->count <= min_tol)) continue;

        NPREdge* e = &*ei;
        std::vector<NPRVertex*> vertices;
        //e->s()->marked = true;
        real len = 0.0;
        NPREdge* prev = e;
        do {
            vertices.push_back(e->s());
            e->visited = true;
            e->twin()->visited = true;
            len += e->length();
            prev = e;
            e = nextFeatureEdge(e);
        } while (e != &*ei && e != NULL);
        if (e != NULL)
            vertices.push_back(e->s());
        e->s()->marked = true;
        if (ei->isProp(EP_CIRCLE)) {
            vertices[vertices.size()/2]->marked = true;
            if (len/dist > 4) {
                vertices[vertices.size()/4]->marked = true;
                vertices[vertices.size()*3/4]->marked = true;
            }
            if (len/dist > 8) {
                vertices[vertices.size()*1/8]->marked = true;
                vertices[vertices.size()*3/8]->marked = true;
                vertices[vertices.size()*5/8]->marked = true;
                vertices[vertices.size()*7/8]->marked = true;
            }
        } else {
            subSample(vertices, 0, vertices.size()-1, len, dist);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::subSample(std::vector<NPRVertex*>& vertices, int begin, int end, real len, real dist)
{
    if (len < dist) return;
    if (begin >= end) return;
    int mid = (begin + end)/2;
    ASSERT(mid < vertices.size());
    vertices[mid]->marked = true;
    subSample(vertices, begin, mid, len/2.0, dist);
    subSample(vertices, mid, end, len/2.0, dist);
}


/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::OnBlend()
{
	// TODO: Add your command handler code here
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::saveColorMap(FILE* fout, NPRMesh& mesh)
{
    int num_samples = 1000;
    int num_vertices = mesh.nVertices();
    Array<NPRVertex*> v_array(num_vertices, NULL);
    int count = 0;
    foreach (vi, mesh.vertices(), NPRMesh::Vertices) {
        v_array[count] = &*vi;
        count++;
    }

    fprintf_s(fout, "%d\r\n", num_samples);
    for (int i=0; i<num_samples; i++) {
        int index = rand() % num_vertices;
        NPRVertex* v = v_array[index];
        if (v->isProp(VP_ANCHOR)) {
            i--;
            continue;
        }
        Color3d color = (*(v->edge_circulator()))->face()->color();
        fprintf_s(fout, "%f %f %f  %f %f %f\r\n",
            v->p().x(), v->p().y(), v->p().z(),
            color.r(), color.g(), color.b());
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::loadColorMap(FILE* fin, NPRMesh& mesh)
{
    int num_samples;
    if (fscanf_s(fin, " %d ", &num_samples) == EOF) {
        CONS("ERROR: unable to parse number of samples.\r\n");
    }

    for (int i=0; i<num_samples; i++) {
        float x, y, z, r, g, b;
        if (fscanf_s(fin, " %f %f %f %f %f %f ", &x, &y, &z, &r, &g, &b) == EOF) {
            CONS("ERROR: unable to parse sample point.\r\n");
        }
        Point3D dummy;
        NPRFace* f = getNearestPoint(Point3D(x, y, z), mesh, dummy);
        if (f != NULL) {
            floodColor(f, Color3d(r, g, b));
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

void CNPRAlg::floodColor(NPRFace* f, const Color3d& color)
{
    ASSERT(f != NULL);
    List<NPRFace*> f_list;
    f_list.push_back(f);
    int min_tol = m_drawerB.getMinTol();

    while (!f_list.empty()) {
        NPRFace* face = f_list.front();
        f_list.pop_front();
        if (face->color() == color) continue;
        face->color() = color;

        foreach (ei, face->edges(), NPRFace::Edges) {
            NPREdge* e = *ei;
            if (e != NULL && e->twin() != NULL) {
                if (e->count > min_tol || e->twin()->count > min_tol)
                    continue;
                if (e->twin()->face()->color() == color) continue;
                f_list.push_back(e->twin()->face());
            }
        }
    }
}

