// Mars lander simulator
// Version 1.10
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2017

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.


//Please read the note at the beginning of void autopilot (void) 

#include "lander.h"
#include<cmath>

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
  // INSERT YOUR CODE HERE

		//NOTE 
		// 1. --- EXTENSION ONE
		//Press 6 to run extension one (scenario 6)


		// 2. --- EXTENSION TWO
		//My autopilot works in all scenario with infinite fuel
		// i turned the autopilot to work with limited fuel in scenario 1 and 5
		// but if you want to test scenario 1 and 5 with limited fuel, you need to change kh!
		double Kp, Kh;
		Kh = 0.001;  //change to 0.017 if you want to test scenario 1 and 5 with limited fuel , kh = 0.001 works for unlimited fuel
		Kp = 1.5;    


		// 3. --- EXTENSION THREE
		//With autopilot disengaged, press 'x','X'   or   'y','Y'    or    'z''Z'   to rotate the pilot


		// 4. --- EXTENSION FOUR&FIVE
		// Owing to limited time, my autopilot ONLY works very well in scenario 9 WITH UNLIMITED FUEL--- lanuch vertically from 10km 
		// The autopilot works in this way
		// I.   Scenario 9 starts with autopilot engaged and executes orbit injection --- the lander will go up and start to tilt slowly
		// II.  When the lander reaches the target orbit (at 100km above the surface), it will remain at that circular orbit, 
		//      and wait for next instruction.
		// III. Then if you press 'd' or 'D', the autopilot will start de-oribiting and landing process.


		// I don't have time to try other extensions.






		//**************************************************************
		// Innovative features of my solution to EXTENSION 4&5
		//****************************************************************


		// I. ---<<Orbit injection>>---
		// I spent lots of time on this topic. Finally, I used the control theory to 'land' the lander to a orbit 
		// with a certain altitude.
		// The idea for this algorithm is very straightforward:
		// If we ignore the rotation of the lander around the Mars during orbit injection,
		// then orbit injection is simply an 'inverse' landing process --- instead of landing to the ground,
		// the lander will 'land' to a specific altitude.
		// Now, if we take rotation around the Mars into consideration, 
		// we need to consider centrifugal force when we calculate the delta term in the control theory equations
		// i tackled this by making a force analysis to the lander and calculate the net force component in position.norm() direction
		// that equals to the magnitude of delta term. 

		// II. ---<<Deorbiting and landing process>>--- 
		// My solution of deorbiting is very clumsy and not very efficient: I first decelerate the lander 
		// by pointing its bottom to velocity direction.
		// Then when the velocity is almost zero, the lander starts the descent --- like a vertical descent.
		// For the landing algorithm, I simply use the control theory mentioned in the handout




		double delta_diff_dec, Pout, eee, mass_of_lander_current,
		altitude_auto, target_rate, r_rate, strange, haha,
		turn_angle;

		mass_of_lander_current = UNLOADED_LANDER_MASS + (fuel*FUEL_CAPACITY*FUEL_DENSITY);//FUEL IS the proportion of fuel remaining 
		delta_diff_dec = GRAVITY * MARS_MASS * mass_of_lander_current * (1 / position.abs2());
		altitude_auto = position.abs() - MARS_RADIUS;
		target_rate = -(0.5 + Kh * altitude_auto);
		r_rate = velocity * position.norm();
		eee = -(0.5 + Kh * altitude_auto + velocity * position.norm()); ///eee is the error term in the equation
		Pout = Kp * eee;
		haha = delta_diff_dec/ (1.5*MAX_THRUST); //  haha is the delta term in the equation
		strange = 1.0 - haha; 

		// if gravity_turn_start_key is pressed, orbit-injection starts
		if (gravity_turn_start_key == true)
		{	
			//In the following lines of code, I am going to calculate by what angle the lander has to be tilted during the orbit injection
			//This reason for doing this is that: the tilted angle needs to increase overtime until it reach 90 degrees.
			//The idea of the tilting algorithm is simple
			//If the lander start to til when it is 10km above the surface
			//Then the lander needs to tilt slowly 
			//and the angle needs to reach 90 degrees 
			//when the lander reaches 100km --- the target orbit altitude
			//So if we divide 90 degrees by (100km - 10km).
			//we can find increment tilting angle once the lander go up by one meter.
			//Therefore, i record the start altitude (start_position = position.abs())  like 10km
			//			   record the target altitude (static double target_position = target_ori_alt + MARS_RADIUS;) like 100km
			// The above two variable will not be overwritten in the next time step
			// Then I record the current altitude 
			// And i can calculate the tilted angle I need at this altitude.

			counter_global += 1.0; //a counter used to trigger if (counter_global == 1.0)
			static double target_position = target_ori_alt + MARS_RADIUS; // this is the altitude of the orbit we want to lander to reach
			static double start_position; // record the start point --- the point that orbit injection starts
			if (counter_global == 1.0)  // use a if statement to hide 'start_position' and prevent it from being overwritten. 
			{
				start_position = position.abs();
			}
			double current_position = position.abs(); 
			double current_ratio = (current_position - start_position) / (target_position - start_position); 
			double det_phi = 90.0 * current_ratio;   //NOTE THAT  det_phi is in degree, 
			double det_phi_radian = (M_PI / 180.0) * det_phi; // convert into radian
			// you need to somehow detect that the orientation has changed to 90 degrees
			double ninety_degrees_reached_check = position * velocity;
			if (abs(ninety_degrees_reached_check) < 0.001) // meaning that position vector is perpendicular to velocity vector now
															// so 90 degrees reached!
			{
				throttle = 0.0;
				attitude_stabilization_gravity_turn( (M_PI / 180.0)* 90.0 ); //lock the lander in 90 degrees orientation
			}

			else // if 90 degrees hasn't reached, then it means the lander is still in orbit injection process
			{
				throttle_control_autopilot(det_phi_radian); // this function control the throttle needed at this orientation
				attitude_stabilization_gravity_turn(det_phi_radian); //this function changs the tilting angle
			}
		}
		

		if (landing_start_key == true)// after the lander reaches the orbit, if you press the this key, deorbiting and landing start.
		{
			if (velocity.abs2() > 1.0)//decelerate the lander until it is almost still
			{
				stabilized_attitude = false;
				stabilized_attitude_ground_speed = true; //point the lander's ass to its velocity direction
				throttle = 1;
			}
			else //start the decent process if it is almost still 
			{
				throttle = 0;
				stabilized_attitude_ground_speed = false;
				stabilized_attitude = true; //point the lander's ass to the ground 
				state_name = Start; // a bool used to activate the landing process
			}

			//now deorbiting has finished, start vertical descent landing!!!
			if (state_name == Start)
			{	
				if (r_rate < MAX_PARACHUTE_SPEED && altitude_auto <= 2614.922) //open the para when these two conditions meet
				{															   //(so the para wil not be damaged)
					parachute_status = DEPLOYED;							    
				}
				//the following code is based on the control theory equations mentioned on the handout
				if (Pout <= -delta_diff_dec)
				{
					throttle = 0;
				}
				else if (Pout > -haha && Pout < strange) //  haha is the delta term in the equation, strange is (1 - delta)
				{
					throttle = haha + Pout;
				}
				else if (Pout >= strange) 
				{
					throttle = 1;
				}
				else
				{
					throttle = 0;
				}
			}

		}
		// a piece of debugging code for writing a text format file 
		/*
		ofstream outfile;
		outfile.open("test2.txt", ios_base::app);
		outfile << target_rate << ' '<< r_rate << ' '<< altitude_auto <<' '<< x() <<endl;
		*/
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // INSERT YOUR CODE HERE
	vector3d F_upt, F_drag_boat, F_drag_para, Gra_Force, F_net, acce;
	double atm_density, area_boat, area_para, mass_curr;
	 
	area_boat = M_PI * pow(LANDER_SIZE,2); //pi*LANDER_SIZE**2
	area_para = 5.0 * (2.0*LANDER_SIZE);
	atm_density = atmospheric_density(position);
	F_drag_boat = (0.5 * atm_density * DRAG_COEF_LANDER * area_boat * velocity.abs2() * velocity.norm()); //'-' as the force is 
	F_drag_para = (0.5 * atm_density * DRAG_COEF_CHUTE * area_para * velocity.abs2() * velocity.norm());//opposite to direc of v
	F_upt = thrust_wrt_world(); 
	//fuel -= delta_t * (FUEL_RATE_AT_MAX_THRUST*throttle) / FUEL_CAPACITY;
	//so change FUEL_RATE_AT_MAX_THRUST can change how quick the fuel runs out 
	mass_curr = UNLOADED_LANDER_MASS + (fuel*FUEL_CAPACITY*FUEL_DENSITY);//FUEL IS the proportion of fuel remaining 
	Gra_Force = GRAVITY * MARS_MASS * mass_curr * (1 / position.abs2()) * position.norm();
	
	if (parachute_status == DEPLOYED)
	{
		F_net = F_upt - F_drag_boat - F_drag_para - Gra_Force;
	}
	else
	{
		F_net = F_upt - F_drag_boat - Gra_Force;
	}
	acce = F_net / mass_curr;
	


	//Euler algorithm 
	if (algo == Euler) // i think the euler i implemented here is semi-implicit Euler. I don't know how to fix it to normal Euler.
	{
		//The difference with the standard Euler method is that 
		//the semi-implicit Euler method uses vn+1 in the equation for xn+1, 
		//while the Euler method uses vn. 	
		acce = F_net / mass_curr;
		velocity = velocity + delta_t * acce;
		position = position + delta_t * velocity;		
	}


	//Verlet algorithm
	else 
	{
		static vector3d previous_position;
		vector3d new_position;

		if (simulation_time == 0.0)
		{
			// do an Euler to compute the first positon 
			//a = -k * x / m; x = x + dt * v; v = v + dt * a; 
			new_position = position + delta_t * velocity;
			velocity = velocity + delta_t * acce;
		}
		else
		{
			//Do a Verlet to compute all subsequent iterations
			new_position = 2 * position - previous_position + (pow(delta_t, 2)) * acce;
			velocity = (1 / delta_t) * (new_position - position);
		}
		previous_position = position;
		position = new_position;
	}
	

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
  //stabilised in ground speed direction, in order to achieve the deceleration needed in re-entry process
  if (stabilized_attitude_ground_speed) attitude_stabilization_ground_speed();
  //test
  //if (stabilized_gravity_turn) attitude_stabilization_gravity_turn();
  //if (throttle_auto_ascent) throttle_control_autopilot;
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "Extension ONE --- Areostationary orbit";
  scenario_description[7] = "Nothing";
  scenario_description[8] = "10km decent by using Euler algo --- tend out that it is a semi-implicit Euler";
  scenario_description[9] = "Extension 4&5 --- Lanuch from 10km, inject to 100km orbit followed by deoribiting and landering ";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit ---- PRESS 'D' or 'd' to land
	position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = true;
	algo = Verlet;
	state_name = Wait;
	gravity_turn_start_key == false;
	landing_start_key == false;
    break;

  case 1:
    // a descent from rest at 10km altitude ---- PRESS 'D' or 'd' to land
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
	algo = Verlet;
	state_name = Wait;
	gravity_turn_start_key == false;
	landing_start_key == true;
    break;

  case 2:
    // an elliptical polar orbit  ---- PRESS 'D' or 'd' to land
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = true;
	algo = Verlet;
	state_name = Wait;
	gravity_turn_start_key == false;
	landing_start_key == false;
    break;

  case 3: 
    // polar surface launch at escape velocity (but drag prevents escape) ---- PRESS 'D' or 'd' to land
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = true;
	algo = Verlet;
	state_name = Wait;
	gravity_turn_start_key == false;
	landing_start_key == false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy ---- PRESS 'D' or 'd' to land
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = true;
	algo = Verlet;
	state_name = Wait;
	gravity_turn_start_key == false;
	landing_start_key == false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere ---- PRESS 'D' or 'd' to land
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = true;
	algo = Verlet;
	state_name = Wait;
	gravity_turn_start_key == false;
	landing_start_key == false;
    break;

  case 6:
	//Areostationary orbit(similar to Earth's geo­stationary orbit, but for Mars)
	// The mass of Mars being 6.4171×10▲23 kg and the sidereal period 88,642 seconds[3].
	//The synchronous orbit thus has a radius of 20,428 km (12693 mi) from the centre of mass of Mars
	//The above is from: https://en.wikipedia.org/wiki/Areostationary_orbit. 
	//---- PRESS 'D' or 'd' to land
    position = vector3d(20428000.0, 0.0, 0.0); //a radius of 20,428 km (12693 mi) from the centre of mass of Mars
    velocity = vector3d(0.0, 1447.96, 0.0); //sidereal period 88,642 seconds so v=w*r and w=2pi/T, so v=2pi*r/T= 1447.96m/s
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
	algo = Verlet;
    autopilot_enabled = true;
	state_name = Wait;
	gravity_turn_start_key == false;
	landing_start_key == false;
    break;

  case 7:
	// a circular equatorial orbit by using Verlet algo
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
	algo = Verlet;
    autopilot_enabled = false;
	state_name = Wait;
	gravity_turn_start_key == false;
	landing_start_key == false;
    break;

  case 8:
	// 10km decent by using Euler algo
	position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
	algo = Euler;
	state_name = Wait;
	gravity_turn_start_key == false;
	landing_start_key == false;
    break;

  case 9:
	//orbit injection from 10km to 100km followed by deorbiting and landing 
	position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = true;
	algo = Verlet;
	state_name = Wait;
	gravity_turn_start_key == true;
	landing_start_key == false;
    break;
  }
}
