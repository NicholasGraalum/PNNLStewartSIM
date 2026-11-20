#include <stdio.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <raylib.h>

/* -------Assumptions--------
	* all angular calculations are done in radian and not degrees
	* distances and values calculated here are arbitrary and based on the siemens drawing
	  done by theodore toea
	* limb_max and limb_min being based around the height from base to platform and the fore_limb
*/

const double pi = 3.14159265358979323846;
double limb, fore_limb, AB_Dist, a_ang, b_ang, a_radius, b_radius;
double limb_max, limb_min;
struct motor{
	double limb;
	double theta;
};

Eigen::Vector3d topPoints[6];
Eigen::Vector3d basePoints[6];
Eigen::Vector3d top_center;
struct motor motors[6];

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
		printf("a_%d : (%.3f, %.3f, %.3f)\n", i + 1, basePoints[i](0), basePoints[i](1), basePoints[i](2));
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
		printf("motor @ (a_%d, b_%d) limb : %.3f , angle : %.3f\n", i+1, i+1, motors[i].limb, motors[i].theta);
	}
}

// Initialize the stewart platform given the values of the platform, base plate and the limbs
static void initStewart()
{
	// initializing all points of the platform in relation to the platforms frame
	basePoints[0] << a_radius * cos(a_ang), -a_radius * sin(a_ang), 0;
	basePoints[1] << a_radius * cos(a_ang), a_radius * sin(a_ang), 0;
	basePoints[2] << a_radius * sin(a_ang - pi/6), a_radius * cos(a_ang - pi/6), 0;
	basePoints[3] << -a_radius * cos(pi/3 - a_ang), a_radius * sin(pi/3 - a_ang), 0;
	basePoints[4] << -a_radius * cos(pi/3 - a_ang), -a_radius * sin(pi/3 - a_ang), 0;
	basePoints[5] << a_radius * sin(a_ang - pi/6), -a_radius * cos(a_ang - pi/6), 0;

	// initializing all points of the base in relation to the base plates frame
	topPoints[0] << b_radius * cos(b_ang), -b_radius * sin(b_ang), 0;
	topPoints[1] << b_radius * cos(b_ang), b_radius* sin(b_ang), 0;
	topPoints[2] << -b_radius * sin(pi / 6 - b_ang), b_radius * cos(pi/6 - b_ang), 0;
	topPoints[3] << -b_radius * cos(pi / 3 - b_ang), b_radius * sin(pi / 3 - b_ang), 0;
	topPoints[4] << -b_radius * cos(pi / 3 - b_ang), -b_radius * sin(pi / 3 - b_ang), 0;
	topPoints[5] << -b_radius * sin(pi/6 - b_ang), -b_radius * cos(pi/6 - b_ang), 0;

	// initializing all motors with the limb distance from base to platform and angle of forelimb
	for (int i = 0; i < 6; i++)
	{
		motors[i].limb = AB_Dist + fore_limb * sin(-pi/2);
		motors[i].theta = -pi/2;
	}

	// initializing the center of the platform and the maximum and minimum distance limbs can move
	top_center << 0, 0, AB_Dist + fore_limb * sin(-pi / 2);
	limb_max = AB_Dist + fore_limb;
	limb_min = AB_Dist - fore_limb;

	printf("Initialized stewart metrics:\n");
	printf("\tlimb : %.3fmm\n\tforelimb : %.3fmm\n\tmax BP_dist : %.3fmm\n\tmin BP_dist : %.3fmm\n", limb, fore_limb, limb_max, limb_min);
	printf("Base Plate:\n\tradius : %.3f\n\tangle offset : %.3f\n", a_radius, a_ang);
	printf("Platform:\n\tradius : %.3f\n\tangle offset : %.3f\n", b_radius, b_ang);
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
			Eigen::Vector3d AB = B_world - topPoints[i];

			motors[i].limb = AB.norm();
			motors[i].theta += atan(fore_limb / motors[i].limb);

		}
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			motors[i].limb = AB_Dist + fore_limb * sin(-pi / 2);
			motors[i].theta = -pi / 2;
		}
	}
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
	fore_limb = 30;
	b_radius = 106.535;
	b_ang = atan(b_radius / 28.0);
	a_radius = 150.0;
	a_ang = atan(a_radius / 45.0);
	AB_Dist = sqrt(pow(limb, 2.0) + pow(fore_limb, 2.0));

	initStewart();
	calcRotationMatrix();
	printRotationMatrix();
	printMotors();

	InitWindow(1280, 720, "Stewart Platform");

	printPlatformVals();

	Camera3D cam = {};
	cam.position = { 500, 400, 500 };
	cam.target = { 0, 100, 0 };
	cam.up = { 0, 1, 0 };
	cam.fovy = 45.0f;
	cam.projection = CAMERA_PERSPECTIVE;
	SetTargetFPS(60);

	char ang = 'a';
	bool rotate = true;
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
					Eigen::Vector3d A = top_center + r_matrix * topPoints[i];
					int j = (i + 1) % 6;
					Eigen::Vector3d A2 = top_center + r_matrix * topPoints[j];

					DrawLine3D(toRayLibCoord(basePoints[i]), toRayLibCoord(A), RED);					// limbs
					DrawLine3D(toRayLibCoord(basePoints[i]), toRayLibCoord(basePoints[j]), DARKGRAY);	// lines connecting base plate
					DrawLine3D(toRayLibCoord(A), toRayLibCoord(A2), BLUE);								// lines connecting top plate
					DrawSphere(toRayLibCoord(basePoints[i]), 4.0f, DARKGRAY);							//draw base points
					DrawSphere(toRayLibCoord(A), 4.0f, BLUE);											// draw top points
				}
		EndMode3D();
		DrawText(TextFormat("alpha : %.3f beta : %.3f rho : %.3f", alpha, beta, rho), 10, 10, 20, BLACK);
		EndDrawing();
	}

}