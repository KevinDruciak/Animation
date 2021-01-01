#include <fstream>
#include "window.h"
#include "Util/gif.h"
#include "Image/bmp.h"
#include "stdio.h"
#include <thread>
#include <Util/cmdLineParser.h>


using namespace std;
using namespace Ray;
using namespace Util;
using namespace Image;

////////////
// Window //
////////////
const char   Window::KEY_ESCAPE = '\033';

Scene*       Window::scene = NULL;

Mouse        Window::mouse;

int          Window::interpolationType = Interpolation::NEAREST;

int          Window::parametrizationType = RotationParameters::TRIVIAL;

bool         Window::isVisible;

int          Window::frameCount;

Util::Timer  Window::timer;

double       Window::radius;

double       Window::frameRate;

Util::Timer  Window::frameCountTimer;

Point3D      Window::center;

int          Window::_width;

int          Window::_height;

bool		 Window::lightsON;

std::string  Window::SceneName;

bool		 Window::Record;

Timer        Window::RecordingTime;

int			 Window::RecordingFrames;

double	     Window::RecordingSeconds;

double       Window::RecordingFrameTime;

int			 Window::RecordingCount;

std::string  Window::RecordingFile;

std::vector<Scene> Window::RecordingScenes;

bool		 Window::RecordRayTraced;

int		     Window::RecordingrLimit;

double       Window::RecordingcutOff;


int Window::PrintError(int showNoError)
{
	int x, y;
	int e = 1;
	switch (glGetError())
	{
	case GL_NO_ERROR:          e = 0; break;
	case GL_INVALID_ENUM:      std::cout << "Invalid Enum" << std::endl; break;
	case GL_INVALID_VALUE:     std::cout << "Invalid Value" << std::endl; break;
	case GL_INVALID_OPERATION: std::cout << "Invalid Operation" << std::endl; break;
	case GL_STACK_OVERFLOW:    std::cout << "Stack Overflow" << std::endl; break;
	case GL_STACK_UNDERFLOW:   std::cout << "Stack Underflow" << std::endl; break;
	case GL_OUT_OF_MEMORY:     std::cout << "Out of memory" << std::endl; break;
	}
	if (!e && showNoError) std::cout << "No error" << std::endl;
	if (!showNoError && !e) return 0;

	glGetIntegerv(GL_MATRIX_MODE, &x);
	if (x == GL_MODELVIEW)
	{
		glGetIntegerv(GL_MODELVIEW_STACK_DEPTH, &x);
		glGetIntegerv(GL_MAX_MODELVIEW_STACK_DEPTH, &y);
		std::cout << "Modelview Matrix: " << x << " / " << y << std::endl;
	}
	else if (x == GL_PROJECTION)
	{
		glGetIntegerv(GL_PROJECTION_STACK_DEPTH, &x);
		glGetIntegerv(GL_MAX_PROJECTION_STACK_DEPTH, &y);
		std::cout << "Projection Matrix: " << x << " / " << y << std::endl;
	}
	return e;
}

void Window::WriteLeftString(int x, int y, const std::string &str)
{
	GLint vp[4];
	GLint dt = glIsEnabled(GL_DEPTH_TEST);
	GLint lm = glIsEnabled(GL_LIGHTING);
	GLint mm;

	glGetIntegerv(GL_VIEWPORT, vp);
	glGetIntegerv(GL_MATRIX_MODE, &mm);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, vp[2], 0, vp[3], 0, 1);

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glColor4f(0.5, 0.5, 0.5, 1);
	glDisable(GL_TEXTURE_2D);
	glRasterPos2f(x, y);
	for (int i = 0; i < str.size(); i++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, str[i]);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	if (dt) glEnable(GL_DEPTH_TEST);
	if (lm) glEnable(GL_LIGHTING);
	glMatrixMode(mm);
}

void Window::WriteRightString(int x, int y, const std::string &str)
{
	GLint vp[4];
	glGetIntegerv(GL_VIEWPORT, vp);

	WriteLeftString(vp[2] - x - glutBitmapLength(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)str.c_str()), y, str);
}

void Window::SetEvaluators(void)
{
	switch (parametrizationType)
	{
	case RotationParameters::TRIVIAL:
		scene->setKeyFrameEvaluator< TransformationParameter< TrivialRotationParameter > >();
		break;
	case RotationParameters::EULER:
		scene->setKeyFrameEvaluator< TransformationParameter< EulerRotationParameter > >();
		break;
	case RotationParameters::ROTATION:
		scene->setKeyFrameEvaluator< TransformationParameter< MatrixRotationParameter > >();
		break;
	case RotationParameters::SKEW_SYMMETRIC:
		scene->setKeyFrameEvaluator< TransformationParameter< SkewSymmetricRotationParameter > >();
		break;
	case RotationParameters::QUATERNION:
		scene->setKeyFrameEvaluator< TransformationParameter< QuaternionRotationParameter > >();
		break;
	default:
		THROW("unsupported parametrization type: %d", parametrizationType);
	}
}

Image32 Window::TakeSnapshot(void)
{
	Image32 img;

	GLint vp[4];
	glGetIntegerv(GL_VIEWPORT, vp);
	GLfloat* pixels = new GLfloat[vp[2] * vp[3] * 3];
	if (!pixels) {
		return img;
	}
	img.setSize(vp[2], vp[3]);

	glReadBuffer(GL_FRONT);

	glReadPixels(vp[0], vp[1], vp[2], vp[3], GL_RGB, GL_FLOAT, pixels);

	for (int i = 0; i < vp[3]; i++) for (int j = 0; j < vp[2]; j++)
	{
		int idx;
		Pixel32 p;
		idx = 0 + j * 3 + (vp[3] - i - 1)*(vp[2]) * 3; p.r = 255 * pixels[idx];
		idx = 1 + j * 3 + (vp[3] - i - 1)*(vp[2]) * 3; p.g = 255 * pixels[idx];
		idx = 2 + j * 3 + (vp[3] - i - 1)*(vp[2]) * 3; p.b = 255 * pixels[idx];
		img(j, i) = p;
	}
	delete[] pixels;
	return img;
}

void Window::IdleFunction(void)
{
	if (RecordRayTraced)
	{
		if (RecordingCount < RecordingFrames)
		{
			if (RecordingTime.elapsed() >= RecordingCount * RecordingFrameTime)
			{
				RecordingTime.stop();
				timer.stop();
				Image32 temp = scene->rayTrace(_width, _height, RecordingrLimit, RecordingcutOff);
				temp.setFlip(true);
				temp.write("image" + std::to_string(RecordingCount) + ".bmp");
				std::cout << "RayTraced scene: " << RecordingCount << std::endl;
				RecordingCount++;
				RecordingTime.resume();
				timer.resume();
				scene->setCurrentTime(timer.elapsed(), interpolationType);
			}
		}
		else
		{
			GifWriter g;
			GifBegin(&g, RecordingFile.c_str(), _width, _height, RecordingFrameTime * 100.0);
			for (int i = 0; i < RecordingCount; i++)
			{
				string file = "image" + std::to_string(i) + ".bmp";
				Image32 tempImage;
				tempImage.read(file);
				GifWriteFrame(&g, reinterpret_cast<uint8_t*>(tempImage.getData()), _width, _height, RecordingFrameTime * 100.0);
				remove(file.c_str());
			}
			std::cout << "Wrote to file: " << RecordingFile << std::endl;
			GifEnd(&g);
			RecordRayTraced = false;
		}
	}
	if (Record)
	{
		if (RecordingCount < RecordingFrames)
		{
			if (RecordingTime.elapsed() >= RecordingCount * RecordingFrameTime)
			{
				RecordingTime.stop();
				timer.stop();
				TakeSnapshot().write("image" + std::to_string(RecordingCount) + ".bmp");
				std::cout << "Captured frame: " << RecordingCount << std::endl;
				RecordingCount++;
				RecordingTime.resume();
				timer.resume();
				scene->setCurrentTime(timer.elapsed(), interpolationType);
			}
		}
		else
		{
			GifWriter g;
			GifBegin(&g, RecordingFile.c_str(), _width, _height, RecordingFrameTime * 100.0);
			for (int i = 0; i < RecordingCount; i++)
			{
				string file = "image" + std::to_string(i) + ".bmp";
				Image32 tempImage;
				tempImage.read(file);
				GifWriteFrame(&g, reinterpret_cast<uint8_t*>(tempImage.getData()), _width, _height, RecordingFrameTime * 100.0);
				remove(file.c_str());
			}
			std::cout << "Wrote to file: " << RecordingFile << std::endl;
			GifEnd(&g);
			Record = false;
		}
	}

	// Update the parameter values
	scene->setCurrentTime(timer.elapsed(), interpolationType);
	// Just draw the scene again
	if (isVisible) glutPostRedisplay();
}

void Window::VisibilityFunction(int state)
{
	if (state == GLUT_VISIBLE) isVisible = true;
	else if (state == GLUT_NOT_VISIBLE) isVisible = false;
}

//////////////////////////
// Mouse event handlers //
//////////////////////////
void Window::MouseFunction(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		RayShapeIntersectionInfo info;
		Ray3D ray = scene->_globalData.camera.getRay(x, y, _width, _height, 0);
		double result = scene->intersect(ray, info);
		if (result != INFINITY) {
			if (info.material->foo == "button") {
				Window::lightsON = !Window::lightsON;
				scene->_globalData.lightsOn = !scene->_globalData.lightsOn;
			} else if (info.material->foo == "door") {
				if (strstr((SceneName).c_str(), "room1.ray") != NULL)
				{
					GLuint temp = scene->OpenGLTessellationComplexity;
					Camera tempCamera = scene->_globalData.camera;

					Scene::BaseDir = GetFileDirectory("room2.ray");
					SceneName = "room2.ray";
					Scene s;
					ifstream istream;
					istream.open("room2.ray");
					istream >> s;
					Shape::OpenGLTessellationComplexity = temp;
					Window::DisplayFunction();
					scene = &s;
					s.updateBoundingBox();
					s._globalData.camera = tempCamera;
					BoundingBox3D b = scene->boundingBox();
					center = (b[0] + b[1]) / 2;
					radius = (b[0] - b[1]).length() / 2;
					s.initOpenGL();
					glutMainLoop();
				} else
				{
					GLuint temp = scene->OpenGLTessellationComplexity;
					Camera tempCamera = scene->_globalData.camera;

					Scene::BaseDir = GetFileDirectory("room1.ray");
					SceneName = "room1.ray";
					Scene s;
					ifstream istream;
					istream.open("room1.ray");
					istream >> s;
					Shape::OpenGLTessellationComplexity = temp;
					Window::DisplayFunction();
					scene = &s;
					s.updateBoundingBox();
					s._globalData.camera = tempCamera;
					BoundingBox3D b = scene->boundingBox();
					center = (b[0] + b[1]) / 2;
					radius = (b[0] - b[1]).length() / 2;
					s.initOpenGL();
					glutMainLoop();
				}
			}
		}

	}
	mouse.update(button, state, x, y);
}

void Window::MotionFunction(int x, int y)
{
	Point2D d = mouse.move(x, y);

	if (mouse.middleDown || (mouse.leftDown && mouse.ctrlDown)) scene->_globalData.camera.rotateUp(center, 0.01*d[0]), scene->_globalData.camera.rotateRight(center, 0.01*d[1]);
	else if (mouse.scrollDown || (mouse.leftDown && mouse.shiftDown)) scene->_globalData.camera.moveForward(radius / 15 * d[1]);
	else if (mouse.leftDown) scene->_globalData.camera.moveRight(-radius / 250 * d[0]), scene->_globalData.camera.moveUp(radius / 250 * d[1]);

	glutPostRedisplay();
}

void Window::PassiveMotionFunction(int x, int y)
{
	mouse.move(x, y);
	glutPostRedisplay();
}

/////////////////////////////
// Keyboard event handlers //
/////////////////////////////
void Window::KeyboardFunction(unsigned char c, int x, int y)
{
	Image32 img;
	std::string imageName;

	switch (c)
	{
	case KEY_ESCAPE:
		exit(0);
		break;
	case 'I':
		std::cout << "Image name: ";
		std::cin >> imageName;
		img = TakeSnapshot();
		img.write(imageName);
		std::cout << "Wrote to file: " << imageName << std::endl;
		break;
	case 'p':
		std::cerr << std::endl;
		std::cerr << "Pos: ( " << scene->_globalData.camera.position << " )" << std::endl;
		std::cerr << "Dir: ( " << scene->_globalData.camera.forward << " )" << std::endl;
		std::cerr << "Up: ( " << scene->_globalData.camera.up << " )" << std::endl;
		break;
	case 'G':
		std::cout << "Frames: ";
		std::cin >> RecordingFrames;
		std::cout << "Seconds: ";
		std::cin >> RecordingSeconds;
		std::cout << "File: ";
		std::cin >> RecordingFile;
		RecordingFrameTime = RecordingSeconds / RecordingFrames;
		RecordingCount = 0;
		Record = true;
		RecordingTime.reset();
		break;
	case 'R':
		std::cout << "Frames: ";
		std::cin >> RecordingFrames;
		std::cout << "Seconds: ";
		std::cin >> RecordingSeconds;
		std::cout << "File: ";
		std::cin >> RecordingFile;
		std::cout << "rLimit: ";
		std::cin >> RecordingrLimit;
		std::cout << "cutOff: ";
		std::cin >> RecordingcutOff;
		RecordingFrameTime = RecordingSeconds / RecordingFrames;
		RecordingCount = 0;
		RecordRayTraced = true;
		RecordingTime.reset();
		break;
	}
}

void Window::SpecialFunction(int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_F1:        break;
	case GLUT_KEY_F2:        break;
	case GLUT_KEY_F3:        break;
	case GLUT_KEY_F4:        break;
	case GLUT_KEY_F5:        break;
	case GLUT_KEY_F6:        break;
	case GLUT_KEY_F7:        break;
	case GLUT_KEY_F8:        break;
	case GLUT_KEY_F9:        break;
	case GLUT_KEY_F10:       break;
	case GLUT_KEY_F11:       break;
	case GLUT_KEY_F12:       break;
	case GLUT_KEY_UP:        break;
	case GLUT_KEY_DOWN:      break;
	case GLUT_KEY_LEFT:      break;
	case GLUT_KEY_RIGHT:     break;
	case GLUT_KEY_PAGE_UP:   break;
	case GLUT_KEY_PAGE_DOWN: break;
	case GLUT_KEY_HOME:      break;
	case GLUT_KEY_END:       break;
	case GLUT_KEY_INSERT:    break;
	}
	glutPostRedisplay();
}

/////////////////////////
// Menu event handlers //
/////////////////////////
void Window::DrawModeMenu(int entry)
{
	glPolygonMode(GL_FRONT_AND_BACK, entry);
	glutPostRedisplay();
}

void Window::CullModeMenu(int entry)
{
	if (entry == NO_CULL) glDisable(GL_CULL_FACE);
	else
	{
		glEnable(GL_CULL_FACE);
		if (entry == CULL_BACK_FACE) glCullFace(GL_BACK);
		if (entry == CULL_FRONT_FACE) glCullFace(GL_FRONT);
	}
	glutPostRedisplay();
}

void Window::InterpolationTypeMenu(int entry)
{
	interpolationType = entry;
	timer = Util::Timer();
	glutPostRedisplay();
}

void Window::ParametrizationTypeMenu(int entry)
{
	parametrizationType = entry;
	SetEvaluators();
	timer = Util::Timer();
	glutPostRedisplay();
}

void Window::MainMenu(int entry)
{
	if (!entry) ERROR_OUT("No entry selected");
	switch (entry)
	{
	case QUIT: exit(0);
	case RAY_TRACE:
	{
		string fileName;
		int recursionDepth;
		double cutOff;
		RayTracingStats::Reset();
		cout << "File name: "; cin >> fileName;
		cout << "Recursion depth: "; cin >> recursionDepth;
		cout << "Cut off: "; cin >> cutOff;
		Image32 img = scene->rayTrace(_width, _height, recursionDepth, cutOff);
		img.write(fileName);
		break;
	}
	case WRITE_SCENE:
	{
		string fileName;
		cout << "File name: "; cin >> fileName;
		ofstream ostream;
		ostream.open(fileName);
		if (!ostream) THROW("Failed to open file for writing: %s\n", fileName.c_str());
		ostream << *scene;
		break;
	}
	default:
		WARN("unrecognized menu option: %d", entry);
	}
}

void Window::DisplayFunction(void)
{
	{
		static bool firstTime = true;
		if (firstTime)
		{
			lightsON = true;
			scene->updateBoundingBox();
			scene->initOpenGL();
		}
		firstTime = false;
	}

	char temp[500];

	// Clear the color and depth buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	float d = radius + (scene->_globalData.camera.position - center).length();

	// Draw the perspective projection (to get good front and back clipping planes, we need to know the rough
	// size of the model)
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(scene->_globalData.camera.heightAngle * 180. / Pi, (double)_width / _height, .1*d, 2 * d);
	if (scene->_globalData.shader && scene->_globalData.shader->glslProgram)
	{
		static bool firstTime = true;
		GLdouble projection[16];
		glGetDoublev(GL_PROJECTION_MATRIX, projection);
		scene->_globalData.shader->glslProgram->setUniformMatrix< 4 >("projection", projection, firstTime);
		firstTime = false;
	}

	// Draw the RayScene
	GLint drawMode[2];
	glGetIntegerv(GL_POLYGON_MODE, drawMode);
	if (drawMode[0] == GL_FILL) scene->drawOpenGL();
	else
	{
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glPolygonOffset(1, 2);
		glDisable(GL_BLEND);
		scene->drawOpenGL();
		glDisable(GL_POLYGON_OFFSET_FILL);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glPolygonMode(GL_FRONT_AND_BACK, drawMode[0]);
		glDisable(GL_BLEND);
		glEnable(GL_LINE_SMOOTH);
		glLineWidth(2);
		scene->drawOpenGL();
	}

	glUseProgram(0);

	// Update the frame rate value on the tenth frame
	frameCount++;
	if (frameCount == 10)
	{
		frameCount = 0;
		frameRate = 10. / frameCountTimer.elapsed();
		frameCountTimer = Util::Timer();
	}
	sprintf(temp, "%.1f fs", frameRate);
	WriteLeftString(1, 2, temp);

	// Write out the mouse position
	sprintf(temp, "( %3d , %3d )", mouse.endX, mouse.endY);
	WriteRightString(1, 2, temp);

	if (scene->_globalData.shader && scene->_globalData.shader->glslProgram) scene->_globalData.shader->glslProgram->use();

	// Swap the back and front buffers
	glutSwapBuffers();
}

void Window::ReshapeFunction(int width, int height)
{
	_width = width, _height = height;
	GLint viewPort[4];

	glViewport(0, 0, width, height);
	glGetIntegerv(GL_VIEWPORT, viewPort);
	glutPostRedisplay();
}

void Window::UpdateView(Scene &s, int width, int height)
{
	scene = &s;
	s.updateBoundingBox();
	BoundingBox3D b = scene->boundingBox();
	center = (b[0] + b[1]) / 2;
	radius = (b[0] - b[1]).length() / 2;
}

void Window::View(Scene &s, int width, int height)
{
	scene = &s;

	SetEvaluators();
	s.setCurrentTime(0, interpolationType);
	s.updateBoundingBox();
	_width = width, _height = height;
	int drawMenu;
	int cullMenu;
	int interpolationTypeMenu;
	int parametrizationTypeMenu;
	timer = Util::Timer();
	BoundingBox3D b = scene->boundingBox();
	center = (b[0] + b[1]) / 2;
	radius = (b[0] - b[1]).length() / 2;
	frameCountTimer = Util::Timer();

	// Initialize the OpenGL context
	int argc = 1;
	char *argv = new char[strlen("hello") + 1];
	strcpy(argv, "hello");
	glutInit(&argc, &argv);
	delete[] argv;

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow("OpenGL Visualization");
	if (glewInit() != GLEW_OK) ERROR_OUT("glewInit failed");
	glutInitWindowSize(width, height);
	glutInitWindowPosition(0, 0);
	glClearColor(0, 0, 0, 1.);
	glutReshapeWindow(width, height);

	// Initialize the event handlers
	glutDisplayFunc(DisplayFunction);
	glutReshapeFunc(ReshapeFunction);
	glutKeyboardFunc(KeyboardFunction);
	glutSpecialFunc(SpecialFunction);
	glutMouseFunc(MouseFunction);
	glutMotionFunc(MotionFunction);
	glutPassiveMotionFunc(PassiveMotionFunction);
	glutVisibilityFunc(VisibilityFunction);
	glutIdleFunc(IdleFunction);

	// Initialize the menus
	drawMenu = glutCreateMenu(DrawModeMenu);
	glutAddMenuEntry(" Polygons ", GL_FILL);
	glutAddMenuEntry(" Lines ", GL_LINE);
	glutAddMenuEntry(" Points ", GL_POINT);

	cullMenu = glutCreateMenu(CullModeMenu);
	glutAddMenuEntry(" None ", NO_CULL);
	glutAddMenuEntry(" Back ", CULL_BACK_FACE);
	glutAddMenuEntry(" Front ", CULL_FRONT_FACE);

	interpolationTypeMenu = glutCreateMenu(InterpolationTypeMenu);
	for (int i = 0; i < Interpolation::COUNT; i++) glutAddMenuEntry(Interpolation::Names[i].c_str(), i);

	parametrizationTypeMenu = glutCreateMenu(ParametrizationTypeMenu);
	for (int i = 0; i < RotationParameters::COUNT; i++) glutAddMenuEntry(RotationParameters::Names[i].c_str(), i);

	glutCreateMenu(MainMenu);
	glutAddSubMenu(" Polygon mode ", drawMenu);
	glutAddSubMenu(" Cull mode ", cullMenu);
	glutAddSubMenu(" Interpolation type ", interpolationTypeMenu);
	glutAddSubMenu(" Rotation parametrization type ", parametrizationTypeMenu);
	glutAddMenuEntry(" Ray-trace ", RAY_TRACE);
	glutAddMenuEntry(" Write scene ", WRITE_SCENE);
	glutAddMenuEntry(" Quit ", QUIT);

	// Attach the drop-down menu to the right button
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glDepthMask(GL_TRUE);
	glDisable(GL_BLEND);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);

	// Fall into the main loop
	glutMainLoop();
}