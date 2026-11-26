#include <stdio.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>

/* -------Assumptions--------
	* all angular calculations are done in radian and not degrees
	* distances and values calculated here are arbitrary and based on the siemens drawing
	  done by theodore toea
	* limb_max and limb_min being based around the height from base to platform and the fore_limb
*/

// Function decleartions
static void initMotors();
static void initStewart();
static void calcRotationMatrix();
static void calcMotorRotation(double theta_offset);
static void step(char ang, double step);
Vector3 toRayLibCoord(const Eigen::Vector3d& v);

// constansts and global variables initialized and defined at the start of int main
const double pi = 3.14159265358979323846;
double AB_Dist, base_ang, top_ang, base_radius, top_radius;
double limb, fore_limb, fore_ang,limb_max, limb_min;

// Points for the platform and base as well as the motors and center of the top platform  
Eigen::Vector3d topPoints[6];
Eigen::Vector3d basePoints[6];
Eigen::Vector3d foreLimbs[6];
Eigen::Vector3d top_center;

struct motor {
	double limb = 0.0;
	double theta = 0.0;
};
struct motor motors[6];

// rotations for the platform in relation to it's frame and the relating rotation matrix
double alpha = 0.0, beta = 0.0, rho = 0.0;
Eigen::Matrix3d r_matrix = Eigen::Matrix3d::Identity();

static void printPlatformVals()
{
	printf("---------------- TOP PLATE ----------------\n");
	for (int i = 0; i < 6; i++)
	{
		printf("b_%d : (%.3f, %.3f, %.3f)\n", i + 1, topPoints[i](0), topPoints[i](1), topPoints[i](2));
	}
	printf("--------------- BASE PLATE ----------------\n");
	for (int i = 0; i < 6; i++)
	{
		printf("a_%d : (%.3f, %.3f, %.3f)\n", i + 1, basePoints[i].x(), basePoints[i].y(), basePoints[i].z());
	}
}

static void printRotationMatrix()
{
	printf("---------------- Rotation Matrix ----------------\n");
	for (int i = 0; i < 9; i++)
	{
		if ((i % 3) == 0) printf("|");
		printf(" %.3f | ", r_matrix(i/3, i % 3));
		if (((i + 1) % 3) == 0) printf("\n");
	}
}

static void printMotors()
{
	printf("---------------- Motors ----------------\n");
	for (int i = 0; i < 6; i++)
	{
		printf("motor @ (a_%d, b_%d) limb : %.3f , angle : %.3f , ", i+1, i+1, motors[i].limb, motors[i].theta);
		printf("fore_limb : %.3f %.3f %.3f\n", foreLimbs[i].x(), foreLimbs[i].y(), foreLimbs[i].y());
	}
}

// set the motors back to the original position, based on which way the forelimb should face
static void initMotors()
{
	for (int i = 0; i < 6; i++)
	{
		double offset = 0.0;
		if (i == 0 || i == 1) offset = pi / 6;
		else if (i == 2 || i == 5) offset = 0.0;
		else offset = pi / 3;

		motors[i].theta = 0.0;
		motors[i].limb = AB_Dist + fore_limb * sin(motors[i].theta);


		
	}
}

// Initialize the stewart platform given the values of the platform, base plate and the limbs
static void initStewart()
{
	basePoints[0] << base_radius * sin(base_ang - pi / 6), -base_radius * cos(base_ang - pi / 6), 0;
	basePoints[1] << base_radius * sin(base_ang - pi / 6), base_radius * cos(base_ang - pi / 6), 0;
	basePoints[2] << base_radius * cos(base_ang), base_radius* sin(base_ang), 0;
	basePoints[3] << -base_radius * cos(pi / 3 - base_ang), -base_radius * sin(pi / 3 - base_ang), 0;
	basePoints[4] << -base_radius * cos(pi / 3 - base_ang), base_radius * sin(pi / 3 - base_ang), 0;
	basePoints[5] << base_radius * cos(base_ang), -base_radius * sin(base_ang), 0;

	topPoints[0] << -top_radius * sin(pi / 6 - top_ang), -top_radius * cos(pi / 6 - top_ang), 0;
	topPoints[1] << -top_radius * sin(pi / 6 - top_ang), top_radius * cos(pi / 6 - top_ang), 0;
	topPoints[2] << top_radius * cos(top_ang), top_radius* sin(top_ang), 0;
	topPoints[3] << -top_radius * cos(pi / 3 - top_ang), -top_radius * sin(pi / 3 - top_ang), 0;
	topPoints[4] << -top_radius * cos(pi / 3 - top_ang), top_radius * sin(pi / 3 - top_ang), 0;
	topPoints[5] << top_radius * cos(top_ang), -top_radius * sin(top_ang), 0;

	foreLimbs[0] << (basePoints[0].x() / abs(basePoints[0].x())) * cos(fore_ang - pi / 2) * fore_limb, (basePoints[0].y() / abs(basePoints[0].y()))* sin(fore_ang - pi / 2)* fore_limb, 0.0;
	foreLimbs[1] << (basePoints[1].x() / abs(basePoints[1].x())) * cos(fore_ang - pi / 2) * fore_limb, (basePoints[1].y() / abs(basePoints[1].y()))* sin(fore_ang - pi / 2)* fore_limb, 0.0;
	foreLimbs[2] << (basePoints[2].x() / abs(basePoints[2].x())) * cos(fore_ang - (3 * pi) / 2) * fore_limb, (basePoints[2].y() / abs(basePoints[2].y()))* sin(fore_ang - (3 * pi) / 2)* fore_limb, 0.0;
	foreLimbs[3] << (basePoints[3].x() / abs(basePoints[3].x())) * cos((5 * pi) / 6 - fore_ang) * fore_limb, (basePoints[3].y() / abs(basePoints[4].y()))* sin((5 * pi) / 6 - fore_ang)* fore_limb, 0.0;
	foreLimbs[4] << (basePoints[4].x() / abs(basePoints[4].x())) * cos((5 * pi) / 6 - fore_ang) * fore_limb, (basePoints[4].y() / abs(basePoints[4].y()))* sin((5 * pi) / 6 - fore_ang)* fore_limb, 0.0;
	foreLimbs[5] << (basePoints[5].x() / abs(basePoints[5].x())) * cos(fore_ang - (3 * pi) / 2) * fore_limb, (basePoints[5].y() / abs(basePoints[5].y()))* sin(fore_ang - (3 * pi) / 2)* fore_limb, 0.0;

	initMotors();

	top_center << 0, 0, AB_Dist;
	limb_max = AB_Dist + fore_limb;
	limb_min = AB_Dist - fore_limb;

	printf("Initialized stewart metrics:\n");
	printf("\tlimb : %.3fmm\n\tforelimb : %.3fmm\n\tmax BP_dist : %.3fmm\n\tmin BP_dist : %.3fmm\n", limb, fore_limb, limb_max, limb_min);
	printf("Base Plate:\n\tradius : %.3f\n\tangle offset : %.3f\n", base_radius, base_ang);
	printf("Platform:\n\tradius : %.3f\n\tangle offset : %.3f\n", top_radius, top_ang);
}

static void calcForeLimbPos()
{
	foreLimbs[0] << (basePoints[0].x() / abs(basePoints[0].x())) * cos(fore_ang - pi / 2) * fore_limb, (basePoints[0].y() / abs(basePoints[0].y()))* sin(fore_ang - pi / 2)* fore_limb, 0.0;
	foreLimbs[1] << (basePoints[1].x() / abs(basePoints[1].x())) * cos(fore_ang - pi / 2) * fore_limb, (basePoints[1].y() / abs(basePoints[1].y()))* sin(fore_ang - pi / 2)* fore_limb, 0.0;
	foreLimbs[2] << (basePoints[2].x() / abs(basePoints[2].x())) * cos(fore_ang - (3 * pi) / 2) * fore_limb, (basePoints[2].y() / abs(basePoints[2].y()))* sin(fore_ang - (3 * pi) / 2)* fore_limb, 0.0;
	foreLimbs[3] << (basePoints[3].x() / abs(basePoints[3].x())) * cos((5 * pi) / 6 - fore_ang) * fore_limb, (basePoints[3].y() / abs(basePoints[4].y()))* sin((5 * pi) / 6 - fore_ang)* fore_limb, 0.0;
	foreLimbs[4] << (basePoints[4].x() / abs(basePoints[4].x())) * cos((5 * pi) / 6 - fore_ang) * fore_limb, (basePoints[4].y() / abs(basePoints[4].y()))* sin((5 * pi) / 6 - fore_ang)* fore_limb, 0.0;
	foreLimbs[5] << (basePoints[5].x() / abs(basePoints[5].x())) * cos(fore_ang - (3 * pi) / 2) * fore_limb, (basePoints[5].y() / abs(basePoints[5].y()))* sin(fore_ang - (3 * pi) / 2)* fore_limb, 0.0;

	for (int i = 0; i < 6; i++)
	{
		foreLimbs[i].x() = cos(motors[i].theta) * foreLimbs[i].x();
		foreLimbs[i].y() = cos(motors[i].theta) * foreLimbs[i].y();
		foreLimbs[i].z() = sin(motors[i].theta) * fore_limb;
	}
}

static void calcMotorRotation(double theta_step, int i)
{
	if ((i + 1) % 2)
	{
		if (motors[i].theta + theta_step > pi / 2) motors[i].theta = pi - motors[i].theta + theta_step;
		else if (motors[i].theta + theta_step < 0)
		{
			motors[i].theta = 0;
			printf("MAX @ MOTOR %d REACHED, alpha : %.3f beta : %.3f rho : %.3f\n", i, alpha, beta, rho);
		}
		else motors[i].theta += theta_step;
	}
	else
	{
		if (motors[i].theta - theta_step < pi / 2) motors[i].theta = pi + motors[i].theta - theta_step;
		else if (motors[i].theta - theta_step > pi)
		{
			motors[i].theta = pi;
			printf("MAX @ MOTOR %d REACHED, alpha : %.3f beta : %.3f rho : %.3f\n", i, alpha, beta, rho);
		}
		else motors[i].theta -= theta_step;
	}
}

static void calcRotationMatrix()
{
	r_matrix << cos(beta) * cos(rho), sin(alpha)* sin(beta)* cos(rho) - cos(alpha) * sin(rho), cos(alpha)* sin(beta)* cos(rho) + sin(alpha) * sin(rho),
				cos(beta) * sin(rho), sin(alpha) * sin(beta) * sin(rho) + cos(alpha) * cos(rho), cos(alpha) * sin(beta) * sin(rho) - sin(alpha) * cos(rho),
				-sin(beta), sin(alpha) * cos(beta), cos(alpha) * cos(beta);
	if (!r_matrix.isApprox(Eigen::Matrix3d::Identity(), 1e-9))
	{
		for (int i = 0; i < 6; i++)
		{
			Eigen::Vector3d B_world = top_center + r_matrix * topPoints[i];
			Eigen::Vector3d AB = B_world - basePoints[i];
			motors[i].limb = AB.norm();
			//calcMotorRotation(atan(fore_limb / motors[i].limb), i);

		}
	}
	else
	{
		initMotors();
	}

	calcForeLimbPos();
}

static void step(char ang, double step)
{
	switch (ang)
	{
	case 'a':
		alpha += step;
		break;
	case 'b':
		beta += step;
		break;
	case 'r':
		rho += step;
		break;
	}
	calcRotationMatrix();
}

Vector3 toRayLibCoord(const Eigen::Vector3d& v)
{
	return { (float)v.x(), (float)v.z(), -(float)v.y() };
}


int main(int argc, char** argv)
{
	limb = 200;
	fore_limb = 55;
	top_radius = 106.535;
	top_ang = atan(top_radius / 28.0);
	base_radius = 150.0;
	base_ang = atan(base_radius / 45.0);
	AB_Dist = sqrt(pow(limb, 2.0) - pow(fore_limb, 2.0));

	fore_ang = pi / 3;//(pi - (2 * pi - (3* base_ang))/3)/2;

	InitWindow(1280, 720, "Stewart Platform");

	initStewart();
	calcRotationMatrix();
	printRotationMatrix();
	printMotors();
	printPlatformVals();

	Camera3D cam = {};
	cam.position = { 500, 400, 500 };
	cam.target = { 0, 100, 0 };
	cam.up = { 0, 1, 0 };
	cam.fovy = 45.0f;
	cam.projection = CAMERA_PERSPECTIVE;
	SetTargetFPS(60);

	char ang = 'a';
	bool rotate = false;
	while (!WindowShouldClose())
	{
		if (IsKeyPressed(KEY_SPACE)) rotate = !rotate;
		if (rotate) UpdateCamera(&cam, CAMERA_ORBITAL);
		else UpdateCamera(&cam, CAMERA_PERSPECTIVE);

		if (IsKeyPressed(KEY_A)) ang = 'a';
		if (IsKeyPressed(KEY_B)) ang = 'b';
		if (IsKeyPressed(KEY_R)) ang = 'r';
		
		if (IsKeyPressed(KEY_EQUAL)) step(ang, 0.1);
		if (IsKeyPressed(KEY_MINUS)) step(ang, -0.1);

		if (IsKeyPressed(KEY_ESCAPE)) CloseWindow();
		BeginDrawing();
		ClearBackground(RAYWHITE);
		BeginMode3D(cam);
			DrawGrid(20, 50.0f);
				for (int i = 0; i < 6; i++)
				{
					Eigen::Vector3d T = top_center + r_matrix * topPoints[i];
					int j = (i + 1) % 6;
					Eigen::Vector3d T2 = top_center + r_matrix * topPoints[j];
					Eigen::Vector3d motor_fore_limb = basePoints[i] + foreLimbs[i];

					DrawLine3D(toRayLibCoord(basePoints[i]), toRayLibCoord(T), RED);					// limbs
					DrawLine3D(toRayLibCoord(basePoints[i]), toRayLibCoord( motor_fore_limb), DARKGREEN);

					DrawLine3D(toRayLibCoord(basePoints[i]), toRayLibCoord(basePoints[j]), DARKGRAY);	// lines connecting base plate
					DrawLine3D(toRayLibCoord(T), toRayLibCoord(T2), BLUE);								// lines connecting top plate

					DrawSphere(toRayLibCoord(basePoints[i]), 4.0f, DARKGRAY);							// draw base points
					DrawSphere(toRayLibCoord(T), 4.0f, BLUE);											// draw top points
				}
		EndMode3D();
		DrawText(TextFormat("alpha : %.3f beta : %.3f rho : %.3f", alpha, beta, rho), 10, 10, 20, BLACK);
		EndDrawing();
	}

}