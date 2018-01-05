#pragma GCC optimize ("Ofast,omit-frame-pointer,inline,unroll-loops")
#pragma GCC option("arch=native","tune=native","no;-zero-upper") // AVX
#pragma GCC target("avx") // AVX

// AUTO VECTORIZATION FOR FOR LOOPS

#include <iostream>
#include <immintrin.h>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <random>
#include <math.h>
#include <time.h>

using namespace std;
using namespace chrono;

// CONSTANTS -------------------------------------------
#pragma region CONSTANTS
constexpr double PI = 3.141592653589793238463;

#define toRad PI / 180
#define toDeg 180 / PI

constexpr int MAX_WIDTH = 16000;
constexpr int MAX_HEIGHT = 9000;
constexpr int MAX_SPEED = 100;

constexpr int Thrust[] = { 0, 50, 100, 150, 200, 650 };
constexpr int Angle[] = { -18, -9, 0, 9, 18 };
#pragma endregion CONSTANTS

// GLOBAL VARIABLES --------------------------------------
#pragma region GLOBAL
int checkpointCount;
int laps;
int myTimeout;
int enemyTimeout;
int eid;    // Enemy Striker ID
int bid;    // Enemy Blocker ID
int sid;    // My Striker ID || !MyStrikerID = My Blocker ID
int variation;

class vec_2d;
class View;
class Pod;
#pragma endregion GLOBAL

// FAST RAND --------------------------------------------
#pragma region RAND
static unsigned int g_seed;
inline void fast_srand(int seed) {
	//Seed the generator
	g_seed = seed;
}
inline int fastrand() {
	//fastrand routine returns one integer, similar output value range as C lib.
	g_seed = (214013 * g_seed + 2531011);
	return (g_seed >> 16) & 0x7FFF;
}
inline int fastRandInt(int maxSize) {
	return fastrand() % maxSize;
}
inline int fastRandInt(int a, int b) {
	return(a + fastRandInt(b - a));
}
inline double fastRandDouble() {
	return static_cast<double>(fastrand()) / 0x7FFF;
}
inline double fastRandDouble(double a, double b) {
	return a + (static_cast<double>(fastrand()) / 0x7FFF)*(b - a);
}
#pragma endregion RAND

// DECLARATIONS -----------------------------------------
#pragma region DECLARATIONS
float collisionTime(vec_2d,vec_2d,vec_2d,vec_2d,float);
#pragma endregion DECLARATIONS

// DATA TYPES -------------------------------------------
#pragma region DATA
class vec_2d
{
public:

	float x, y;

	vec_2d() {}
	vec_2d(float x, float y) : x(x), y(y) {}

    // Overloading Operators
	vec_2d operator*(float a) { return vec_2d(x * a, y * a); }
	vec_2d operator/(float a) { return vec_2d(x / a, y / a); }
	vec_2d operator-(vec_2d b) { return vec_2d(x - b.x, y - b.y); }
	vec_2d operator+(vec_2d b) { return vec_2d(x + b.x, y + b.y); }

	bool operator==(vec_2d &rhs)
	{
		return x == rhs.x && y == rhs.y;
	}

	bool operator==(float rhs)
	{
		return x == rhs && y == rhs;
	}

    // Maths Functions
	float Dot(vec_2d b) { return x * b.x + y * b.y; }   // Dot Product
	float Dot2(vec_2d b) { return y * b.x + x * b.y; } 
	float Det(vec_2d b) { return y * b.x - b.y * x; }
	float Len2() { return Dot(*this); } // Length Of Vector
	float Len() { return sqrt(Len2()); }
	float Distance2(vec_2d b) { vec_2d c = *this - b; return c.Len2(); } // C = A - B = New Vector. C length = Distance. 
	float Distance(vec_2d b) { return sqrt(Distance2(b)); }

	float getAngle(vec_2d b) {          // Absolute Angle to point B
		vec_2d c = b - *this;
		float rad = atan2(c.y, c.x);
		float deg = rad * toDeg;
		if (c.y<0.0)
			deg += 360;
		return deg;
	}

	vec_2d closest(vec_2d b) {          // Closes point to another point B
		vec_2d norm = { b.y - y, x - b.x };
		float c = this->Dot(norm);
		float det = norm.Len2();
		return norm * c / det;
	}

	vec_2d Normalize() { return *this * 1.0 / Len(); }  // Normalize 
	vec_2d Round() { return vec_2d(round(x), round(y)); }   // Round-off
	vec_2d Trunc() { return vec_2d(int(x), int(y)); }   // Truncate      


}checkpoint[8];

class View
{
public:

	int angle, thrust;
	View() : angle(0), thrust(0) {};
}bestView[2][6];

class Pod
{
public:

	vec_2d pos, vel;
	int boost, shield, angle, target, timeout, nextId;
	View view[6];

    // Difference to another point B
	int diffAngle(vec_2d b)
	{
		int a = pos.getAngle(b);

		int right = angle <= a ? a - angle : 360.0 - angle + a;
		int left = angle >= a ? angle - a : angle + 360.0 - a;

		if (right < left)
			return right;
		else
			return -left;
	}

    // Rotate current Pod 
	void rotate(int ang)
	{
		if (ang > 18)
			ang = 18;
		if (ang < -18)
			ang = -18;
		angle += ang;

		if (angle > 360)
			angle = angle - 360;
		if (angle < 0)
			angle = 360 + angle;
	}

    // Accelerate Pod
	void thrust(int acc)
	{
		vel.x += cos(angle * toRad) * acc;
		vel.y += sin(angle * toRad) * acc;
	}

    // Compute new direction vector
	vec_2d next(int ang)
	{
		int a = angle;

		if (ang > 18)
			ang = 18;

		if (ang < -18)
			ang = -18;

		a += ang;

		if (a > 360)
			a = a - 360;

		if (a < 0)
			a = a + 360;

		vec_2d n = vel;

		n.x += cos(a * toRad) * 100000;
		n.y += sin(a * toRad) * 100000;

		n = pos + n;

		return n.Round();
	}

    // Update position
	void move(float t)
	{
		if (collisionTime(pos, checkpoint[nextId], vel, vec_2d(0, 0), 600 * 600)<t)
		{
			timeout = 0;
			--target;

			if (nextId == checkpointCount - 1)
			{
				nextId = 0;
			}
			else
			{
				++nextId;
			}
		}

		pos = pos + vel * t;
	}

	void end()
	{
		vel = vel * 0.85;
		pos = pos.Round();
		vel = vel.Trunc();
	}

}pod[4];
#pragma endregion DATA

// MATHS FUNCTIONS ------------------------------------------------------------
#pragma region MATHS_FUNC
inline float Dot(vec_2d a, vec_2d b) { return a.x * b.x + a.y * b.y; }
inline float Dot2(vec_2d a, vec_2d b) { return a.y * b.x + a.x * b.y; }
inline float Det(vec_2d a, vec_2d b) { return a.y * b.x - b.y * a.x; }
inline float Len2(vec_2d vec) { return Dot(vec, vec); }
inline float Len(vec_2d vec) { return sqrt(Len2(vec)); }
inline float Distance2(vec_2d a, vec_2d b) { return Len2(a - b); }
inline float Distance(vec_2d a, vec_2d b) { return sqrt(Distance2(a, b)); }

inline vec_2d Normalize(vec_2d vec) { return vec * 1.0 / Len(vec); }
inline vec_2d Round(vec_2d vec)
{
	return vec_2d(round(vec.x), round(vec.y));
}
inline vec_2d Trunc(vec_2d vec)
{
	return vec_2d(int(vec.x), int(vec.y));
}

float getAngle(vec_2d a, vec_2d b)
{
	vec_2d c = b - a;
	float rad = atan2(c.y, c.x);
	float deg = rad * toDeg;
	if (c.y < 0.0)
		deg += 360;

	return deg;
}

vec_2d intersect(vec_2d p1v1, vec_2d p1v2, vec_2d p2v1, vec_2d p2v2)
{
	vec_2d A = p1v2 - p1v1;
	float c1 = Dot2(A, p1v1);

	vec_2d A2 = p2v2 - p2v1;
	float c2 = Dot2(A2, p2v1);

	float det = Det(A, A2);

	if (det == 0)
	{
		return vec_2d(-1, -1);
	}

	return vec_2d((A2.x * c1 - A.x * c2) / det, (A.y * c2 - A2.y * c1) / det);
}

vec_2d closest(vec_2d p0, vec_2d p1)
{
	vec_2d norm = { p1.y - p0.y, p0.x - p1.x };
	float c = Dot(norm, p0);
	float det = Len2(norm);
	return norm * c / det;
}
#pragma endregion MATHS_FUNC

// COLLISION -----------------------------------------------------------------------
#pragma region COLLISION
inline float collisionTime(vec_2d p1, vec_2d p2, vec_2d v1, vec_2d v2, float sr)
{
	vec_2d dv = v1 - v2;    // Difference in velocity
	vec_2d dp = p1 - p2;    // Difference in Positon

	float a = Dot(dv, dv);  // Magnitude of Velocity
	float b = 2 * Dot(dv, dp);
	
	if (b >= 0)             // No Collision this turn 
	{
		return 1000;
	}

    float c = Dot(dp, dp) - sr; // Distance - Sum of radius

	float d = b*b - 4 * a*c;

	if (d < 0)             // Imaginary Number
	{
		return 1000;
	}

	return (-b - sqrt(d)) / (2 * a);
}

void bounce(Pod &a, Pod &b)
{
	float m1 = a.shield == 3 ? 10 : 1;
	float m2 = b.shield == 3 ? 10 : 1;
	float mcoeff = (m1 + m2) / (m1 * m2);

	vec_2d n = a.pos - b.pos;

	// Square of the distance between the 2 pods. This value could be hardcoded because it is always 800²

	float nsquare = 640000; // Radius is 800

	vec_2d dv = a.vel - b.vel;

	// fx and fy are the components of the impact vector. product is just there for optimisation purposes

	float product = Dot(n, dv);

	float fx = (n.x * product) / (nsquare * mcoeff);
	float fy = (n.y * product) / (nsquare * mcoeff);


	// We apply the impact vector once
	a.vel.x -= fx / m1;
	a.vel.y -= fy / m1;
	b.vel.x += fx / m2;
	b.vel.y += fy / m2;

	// If the norm of the impact vector is less than 120, we normalize it to 120
	float impulse = sqrt(fx * fx + fy * fy);
	if (impulse < 120.0)
	{
		fx = fx * 120.0 / impulse;
		fy = fy * 120.0 / impulse;
	}

	// We apply the impact vector a second time
	a.vel.x -= fx / m1;
	a.vel.y -= fy / m1;
	b.vel.x += fx / m2;
	b.vel.y += fy / m2;
}
#pragma endregion COLLISION

// SIMULATING A TURN --------------------------------------------------------
#pragma region SIMULATE
void simulate(Pod p[])
{
	float t = 0.0;  // Time passed since start of turn

	while (t < 1.0)
	{
		double first = 1.0 - t; // Time before turn ends
		Pod *a, *b;

		for (int i = 0; i < 4; ++i)
		{
			int j = 0;

			while (i != j)
			{
                // Time before pods collide
				float col = collisionTime(p[j].pos, p[i].pos, p[j].vel, p[i].vel, 800 * 800);

                // If collision occurs before previously calculated collision
                // or turn ends
				if (col < first)
				{
					first = col;    // Collision at col time
					a = &p[j];
					b = &p[i];
				}

				++j;
			}
		}

		for (int i = 0; i < 4; ++i)
		{
			p[i].move(first); // Move all pods nomrally till pods collide or turn ends
		}

		if (first != (1.0 - t))
		{
			bounce(*a, *b); // If pods collide bounce
		}

		t += first; // Update time passed
	}

	for (int i = 0; i < 4; ++i)
	{
		p[i].end();
	}
}
#pragma endregion SIMULATE

// SCORING FUNCTIONS
#pragma region SCORE
float strikerScore(Pod test[], int myStriker, int enemyStriker, int enemyBlocker)
{
	float score = Distance2(test[myStriker].pos, checkpoint[test[myStriker].nextId]) * 0.000001 + (1 - Dot(Normalize(checkpoint[test[myStriker].nextId] - test[myStriker].pos), Normalize(test[myStriker].vel))) - abs(test[myStriker].diffAngle(test[enemyBlocker].pos)) * 0.01;

	if (test[myStriker].nextId != pod[myStriker].nextId)
	{
		score -= (checkpointCount * laps - test[myStriker].target) * 1000;
	}

	if (pod[myStriker].target == 0 && test[myStriker].nextId == 1)
	{
		score = -100000000000;
	}

	score -= (test[enemyStriker].target - test[myStriker].target) * 1000;

	return score;
}

float blockerScore(Pod test[], int myStriker, int myBlocker, int enemyStriker, int enemyBlocker)
{
	float score = 0;

	float enemyDistancetoCP = Distance2(test[enemyStriker].pos, checkpoint[test[enemyStriker].nextId]);
	float myDistancetoCP = Distance2(test[myBlocker].pos, checkpoint[test[enemyStriker].nextId]);

	score += myDistancetoCP * 0.00001 - enemyDistancetoCP * 0.000001 + abs(test[myBlocker].diffAngle(test[enemyStriker].pos)) - strikerScore(test, enemyStriker, myStriker, myBlocker) * 10;


	return score;
}
#pragma endregion SCORE

// AI PART ---------------------------------------------------------------------
#pragma region AI
void myPod()
{
	auto startTime = system_clock::now();
	auto endTime = 0;
	variation = 0;
	Pod test[4];

	float bestScore[2] = { 999999999999999, 999999999999999 };

	int sim = 0, count = 0, n;

	do
	{
		n = 0;

		for (int j = 0; j < 4; ++j)
		{
			test[j].pos = pod[j].pos;
			test[j].vel = pod[j].vel;
			test[j].angle = pod[j].angle;
			test[j].nextId = pod[j].nextId;
			test[j].target = pod[j].target;
			test[j].boost = pod[j].boost;
			test[j].shield = pod[j].shield;
		}

		++sim;
		++count;

		for (int j = 0; j < 6; ++j)
		{
			for (int k = 0; k < 2; ++k)
			{
				int sh = rand() % 4;
				int angle = Angle[rand() % 5];
				int thrust;

				if (j == sh && ((Distance2(test[k].pos, test[!k].pos) < 1500 * 1500 || Distance2(test[k].pos, test[eid].pos) < 1500 * 1500 || Distance2(test[k].pos, test[bid].pos) < 1500 * 1500)))
				{
					test[k].shield = 3;
				}

				if (test[k].shield > 0)
				{
					thrust = -1;
				}
				else
					if (test[k].boost)
					{
						thrust = Thrust[rand() % 6];

						if (thrust == 650)
						{
							test[k].boost = 0;
						}
					}
					else
					{
						thrust = Thrust[rand() % 5];
					}

				test[k].rotate(angle);
				test[k].thrust(thrust);

				test[k].view[j].angle = angle;
				test[k].view[j].thrust = thrust;
			}

			int a = test[eid].diffAngle(checkpoint[test[eid].nextId] - test[eid].vel * 4);

			test[eid].rotate(a);
			test[eid].thrust(200);

			a = test[bid].diffAngle(test[sid].pos - test[bid].vel * 4);
			test[bid].rotate(a);
			test[bid].thrust(200);

			simulate(test);

			if (test[0].shield > 0)
			{
				--test[0].shield;
			}
			if (test[1].shield >0)
			{
				--test[1].shield;
			}
		}

		float score;

		if (myTimeout > 70 && myTimeout > enemyTimeout)
		{
			score = strikerScore(test, sid, eid, bid) + strikerScore(test, !sid, eid, bid) - 10000;
		}
		else
		{
			score = strikerScore(test, sid, eid, bid) * 10 + n * 100 + blockerScore(test, sid, !sid, eid, bid);
		}

		if (score < bestScore[0])
		{
			++variation;
			bestScore[0] = score;

			for (int j = 0; j < 6; ++j)
			{
				bestView[sid][j] = test[sid].view[j];
				bestView[!sid][j] = test[!sid].view[j];
			}
		}

		if (count == 100)
		{
			endTime = duration_cast<milliseconds>(system_clock::now() - startTime).count();
			count = 0;
		}

	} while (endTime < 140);

	/*
	cerr << "Sims :" << sim <<'\n';
	cerr << "Time taken :" << duration_cast<milliseconds>(system_clock::now() - startTime).count() << '\n';
	*/
	for (int i = 0; i < 6; ++i)
	{
		pod[sid].view[i] = bestView[sid][i];
		pod[!sid].view[i] = bestView[!sid][i];
	}
}
#pragma endregion AI

//MAIN
int main()
{

	cin >> laps;
	cin.ignore();
	cin >> checkpointCount;
	cin.ignore();

	for (int i = 0; i < 4; ++i)
	{

		pod[i].shield = 0;
		pod[i].boost = 1;
		pod[i].timeout = 0;
		pod[i].nextId = -1;
		pod[i].target = laps * checkpointCount;
	}

	for (int i = 0; i < checkpointCount; ++i)
	{
		cin >> checkpoint[i].x >> checkpoint[i].y;
		cin.ignore();
	}

	// game loop
	while (1)
	{

		for (int i = 0; i < 4; ++i)
		{

			int id = pod[i].nextId;

			cin >> pod[i].pos.x >> pod[i].pos.y >> pod[i].vel.x >> pod[i].vel.y >> pod[i].angle >> pod[i].nextId;
			cin.ignore();

			if (pod[i].angle < 0)
			{
				pod[i].angle = getAngle(pod[i].pos, checkpoint[pod[i].nextId]);
			}

			if (id != pod[i].nextId)
			{
				pod[i].target--;
				pod[i].timeout = 0;
			}

			else
			{
				++pod[i].timeout;
			}
		}

		if (pod[2].target < pod[3].target || pod[2].target == pod[3].target && Distance2(pod[2].pos, checkpoint[pod[2].nextId]) < Distance2(pod[3].pos, checkpoint[pod[3].nextId]))
		{
			eid = 2;
			bid = 3;
		}
		else
		{
			eid = 3;
			bid = 2;
		}

		if (pod[0].target < pod[1].target || pod[0].target == pod[1].target && Distance2(pod[0].pos, checkpoint[pod[0].nextId]) < Distance2(pod[1].pos, checkpoint[pod[1].nextId]))
		{
			sid = 0;
		}
		else
		{
			sid = 1;
		}

		myTimeout = pod[0].timeout<pod[1].timeout ? pod[0].timeout : pod[1].timeout;
		enemyTimeout = pod[2].timeout<pod[3].timeout ? pod[2].timeout : pod[3].timeout;

		myPod();

		for (int i = 0; i < 2; ++i)
		{
			vec_2d next = pod[i].next(pod[i].view[0].angle);

			if (pod[i].view[0].thrust == 650)
			{
				pod[i].boost = 0;
				cout << next.x << " " << next.y << " BOOST BOOST ";
			}
			else if (pod[i].view[0].thrust == -1 && pod[i].shield == 0)
			{
				pod[i].shield = 3;
				cout << next.x << " " << next.y << " SHIELD SHIELD ";
			}
			else if (pod[i].view[0].thrust == -1 && pod[i].shield > 0)
			{
				pod[i].shield--;
				cout << next.x << " " << next.y << " 0 ";
			}
			else
			{
				cout << next.x << " " << next.y << " " << pod[i].view[0].thrust;

				if (i == sid)
				{
					cout << " STRIKER ";
				}
				else
				{
					cout << " BLOCKER ";
				}
			}

			for (int j = 0; j<6; ++j)
			{
				cout << '\t' << bestView[i][j].angle << " " << bestView[i][j].thrust;
			}

			cout << '\t' << variation << endl;
		}
	}
}
