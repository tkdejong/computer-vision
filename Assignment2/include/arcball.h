#ifndef ARCBALL_H
#define ARCBALL_H

/* Arcball, written by Bradley Smith, March 24, 2006
 * arcball.h is free to use and modify for any purpose, with no
 * restrictions of copyright or license.
 *
 * Using the arcball:
 *   Call arcball_setzoom after setting up the projection matrix.
 *
 *     The arcball, by default, will act as if a sphere with the given
 *     radius, centred on the origin, can be directly manipulated with
 *     the mouse. Clicking on a point should drag that point to rest under
 *     the current mouse position. eye is the position of the eye relative
 *     to the origin. up is unused.
 *
 *     Alternatively, pass the value: (-radius/|eye|)
 *     This puts the arcball in a mode where the distance the mouse moves
 *     is equivalent to rotation along the axes. This acts much like a
 *     trackball. (It is for this mode that the up vector is required,
 *     which must be a unit vector.)
 *
 *     You should call arcball_setzoom after use of gluLookAt.
 *     gluLookAt(eye.x,eye.y,eye.z, ?,?,?, up.x,up.y,up.z);
 *     The arcball derives its transformation information from the
 *     openGL projection and viewport matrices. (modelview is ignored)
 *
 *     If looking at a point different from the origin, the arcball will still
 *     act as if it centred at (0,0,0). (You can use this to translate
 *     the arcball to some other part of the screen.)
 *
 *   Call arcball_start with a mouse position, and the arcball will
 *     be ready to manipulate. (Call on mouse button down.)
 *   Call arcball_move with a mouse position, and the arcball will
 *     find the rotation necessary to move the start mouse position to
 *     the current mouse position on the sphere. (Call on mouse move.)
 *   Call arcball_rotate after resetting the modelview matrix in your
 *     drawing code. It will call glRotate with its current rotation.
 *   Call arcball_reset if you wish to reset the arcball rotation.
 */

#include <cmath> // for sqrt

#ifdef _WIN32
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#ifdef __linux__
#include <GL/glut.h>
#include <GL/glu.h>
#endif

class vec  // simple 3D vector class
{
public:
	float x, y, z;

	vec() :
			x(0), y(0), z(0)
	{
	}
	vec(float xx)
	{
		x = xx;
		y = xx;
		z = xx;
	}
	vec(float xx, float yy, float zz)
	{
		x = xx;
		y = yy;
		z = zz;
	}

	inline vec operator +=(vec t)  // self-addition
	{
		*this = *this + t;
		return *this;
	}

	inline vec operator +(vec t)  // addition
	{
		return vec(x + t.x, y + t.y, z + t.z);
	}
	inline vec operator -(vec t)  // subtraction
	{
		return vec(x - t.x, y - t.y, z - t.z);
	}
	inline vec operator *(float t)  // dot product
	{
		return vec(x * t, y * t, z * t);
	}
	inline float operator *(vec t)  // scalar product
	{
		return x * t.x + y * t.y + z * t.z;
	}
	inline vec operator ^(vec t)  // cross product
	{
		return vec(y * t.z - z * t.y, t.x * z - x * t.z, x * t.y - y * t.x);
	}
	inline float length()  // pythagorean length
	{
		return sqrt(x * x + y * y + z * z);
	}
	inline vec unit()  // normalized to a length of 1
	{
		float l = length();
		if (l == 0.0) return vec(0.0, 0.0, 0.0);
		return vec(x / l, y / l, z / l);
	}
	inline bool zero()  // returns true if a zero vector
	{
		return x == 0 && y == 0 && z == 0;
	}
	inline bool equals(vec t)  // returns true if exactly equal
	{
		return x == t.x && y == t.y && z == t.z;
	}
};

extern void arcball_setzoom(float radius, vec eye, vec up);
extern void arcball_rotate();
extern void arcball_add_angle(int);
extern void arcball_add_distance(int);
extern void arcball_reset();
extern void arcball_start(int mx, int my);
extern void arcball_move(int mx, int my);

#endif
