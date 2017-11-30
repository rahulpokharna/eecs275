#include "minimal_turtlebot/turtlebot_controller.h"
#include<ros/ros.h>
#include <cmath>
#include <algorithm>

uint64_t startTime;
int state;
int bump;
float turnAngle;
int turnCount;
float closest;
#define SECONDINNS 1000000000ll
int numNaN;
float goal[2] = {5, 5};
float noSin;


void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel)
 {
	//Place your code here! you can access the left / right wheel 
	//dropped variables declared above, as well as information about
	//bumper status. 
	
	//outputs have been set to some default values. Feel free 
	//to change these constants to see how they impact the robot. 
	ROS_INFO("-----------------------------------------------------------------");
	ROS_INFO("Current x: %f",turtlebot_inputs.x);
	ROS_INFO("Current y: %f",turtlebot_inputs.y);
	ROS_INFO("Current z_angle: %f",turtlebot_inputs.z_angle);
	ROS_INFO("Current orientation_omega: %f",turtlebot_inputs.orientation_omega);
	ROS_INFO("Current State: %i",state);
	ROS_INFO("Current Saved Time: %u",startTime);
	ROS_INFO("Current Shared Time: %u",turtlebot_inputs.nanoSecs);		
	ROS_INFO("Current closest point: %f", closest);
	ROS_INFO("Current angvel: %f", *ang_vel);
	ROS_INFO("Current vel: %f", *vel);
	
	
	if(turtlebot_inputs.rightWheelDropped + turtlebot_inputs.leftWheelDropped > 0){
			state = 3;
	}
	
	
	switch(state){
		//normal drive forward case
		//drive towards the goal position
		// add steering
		case 0:
		{
			//calculate the angle needed to be at for movement to goal
			if(isnan(turtlebot_inputs.y)){
				*ang_vel = 0;
				*vel =  .15; 	
			}
			else{
				float goalAang = 2 * atan2f(goal[1] - turtlebot_inputs.y, goal[0] - turtlebot_inputs.x);
				
				// current angle 
				float currentAang = 2 * atan2f(turtlebot_inputs.z_angle,turtlebot_inputs.orientation_omega);
				
				float tempAangVel = (goalAang - currentAang)/6.3; // rounded 2 pi
				float tempKorraVel = (currentAang - goalAang)/6.3; // rounded 2 pi
				
				if( isnanf(tempAangVel)){ 
					*ang_vel = 0;
				}
				else{
					
					if(tempAangVel > 3.14 || tempAangVel < -3.14){
						*ang_vel = 6.3 - tempAangVel;
					}
					else{
						*ang_vel = tempAangVel;
					}
		
//					if(fabsf(tempAangVel) < fabsf(tempKorraVel)){
	//					*ang_vel = tempAangVel;
		//			}
			//		else{
				//		*ang_vel = tempKorraVel;
					//}
				}
				
				//min between .7 and max(distance and .2)
				float distance =  sqrtf(pow(goal[1] - turtlebot_inputs.y,2) + pow(goal[0] - turtlebot_inputs.x, 2));
				
				//consider normalizing distance according to shortest length
				float normalized = distance / sqrtf(pow(goal[1],2) + pow(goal[0], 2));
				float temp = std::max(normalized, .1f);
				*vel = std::min(.5f, temp);
				//*vel = .4
				
				if(fabsf(goal[1] - turtlebot_inputs.y) < .1 &&  fabsf(goal[0] - turtlebot_inputs.x) < .1){
					state = 6;
					*vel = 0;
					turnCount = 0;
					turnAngle = currentAang;
					startTime = turtlebot_inputs.nanoSecs;
					//turn is circles and then go home
				}
				
				ROS_INFO("Current orientation angle: %f", currentAang * 180 / 3.14);
				ROS_INFO("Current goal angle: %f",goalAang * 180 / 3.14);
				
			}
			
			
			
	
			// arrived at goal 
			
			
			// Replace above with below, scale and use min/max to make sure range is between .5 and 0
			//sqrtf(pow(goal[1] - turtlebot_inputs.y,2) + pow(goal[0] - turtlebot_inputs.x, 2));
			
			for (int i = 0; i < 640; i = i + 10){
				
				
				//this if checks for the closest object within .5m using 1m for the simulation due to position
				if ((turtlebot_inputs.ranges[i] < closest || closest == 0 )&& isnan(turtlebot_inputs.ranges[i]) == false){
					
					closest = turtlebot_inputs.ranges[i];
					// stop and wait here
									
				}
								
				//veer away if it is above .5m	
					
			}
			if(closest < 1 && closest != 0){
					startTime = turtlebot_inputs.nanoSecs;
					state = 4;
					closest = 0;
					numNaN = 0;
					
				}
			
			
			if(turtlebot_inputs.leftBumperPressed + turtlebot_inputs.rightBumperPressed + turtlebot_inputs.centerBumperPressed + turtlebot_inputs.sensor0State + turtlebot_inputs.sensor1State + turtlebot_inputs.sensor2State > 0)
			{
				startTime = turtlebot_inputs.nanoSecs;
				state = 1;
				if(turtlebot_inputs.leftBumperPressed > 0 ||  turtlebot_inputs.sensor0State > 0){
					bump = 1;
				}
				else{
					bump = -1;
				}
				
				
			}
			
		}break;
		//reverse upon bump or cliff
		case 1:
		{
			

			if(fabsf(turtlebot_inputs.nanoSecs - startTime) <= SECONDINNS){
				
				*vel = -.15;
			}
			if(fabsf(turtlebot_inputs.nanoSecs - startTime) > SECONDINNS){
				startTime = turtlebot_inputs.nanoSecs;
				*vel = 0;
				state = 2;
			}
			
		}break;
		//turn for bump or cliff
		case 2:
		{
			if(fabsf(turtlebot_inputs.nanoSecs - startTime) <= 2 * SECONDINNS){
					
				*ang_vel = -.4 * bump;
			}
			if(fabsf(turtlebot_inputs.nanoSecs - startTime) > 2 * SECONDINNS){
				startTime = turtlebot_inputs.nanoSecs;
				*ang_vel = 0;
				state = 0;
				bump = 0;
			}
			
		}break;
		//wheels are dropped/error case
		case 3:
		{
			*ang_vel = 0;
			*vel = 0;
			*soundValue = 4;
			
			if(turtlebot_inputs.rightWheelDropped + turtlebot_inputs.leftWheelDropped == 0){
				state = 0;
				*soundValue = 0;	
			}	
		}break;
		// something is .5 meters from us (1m is simulation)
		case 4:
		{
			*ang_vel = 0;
			*vel = 0;
			*soundValue = 2;
			
			if(fabsf(turtlebot_inputs.nanoSecs - startTime) > 10 * SECONDINNS){
				
				//change state to spin until it finds a point that is "free"
				state = 5;
				*soundValue = 0;
				startTime = turtlebot_inputs.nanoSecs;
			}
			
		}break;
		// rotate until you find a free path to go forward
		case 5:
		{
			*ang_vel = .2;
			*vel = 0;
			numNaN = 0;
			for (int i = 0; i < 640; i = i + 1){
				
				//this if checks for the closest object within .5m using 1m for the simulation due to position
				if ((turtlebot_inputs.ranges[i] < closest || closest == 0 ) && isnan(turtlebot_inputs.ranges[i]) == false){
					
					
						closest = turtlebot_inputs.ranges[i];
					// stop and wait here
					
					
				}
				if(isnan(turtlebot_inputs.ranges[i])||turtlebot_inputs.ranges[i] > 1.2){
						numNaN += 1;
					}
				
					
			}
			if((closest > 1.2 || closest == 0) && numNaN > 200){ // needs to be fixed
				state = 0;
				numNaN = 0;
				*ang_vel = 0;
			}
			closest = 0;
			
		}break;
		case 6:{
			*ang_vel = .5;
			//rotate 4 times
			float currentAang = atan2f(turtlebot_inputs.z_angle,turtlebot_inputs.orientation_omega);
			if(turnCount < 4 && llabs(startTime - turtlebot_inputs.nanoSecs) > SECONDINNS){
				if(fabsf(turnAngle - currentAang) < .05){
					turnCount += 1;
					startTime = turtlebot_inputs.nanoSecs;
				}
			}
			
			if(turnCount >= 4){
				state = 7;
				*ang_vel = 0;
				startTime = turtlebot_inputs.nanoSecs;
			}
			
			ROS_INFO("Current number Turns: %i",turnCount);
			ROS_INFO("Current turn angle: %f",turnAngle * 180 / 3.14);	
			ROS_INFO("Current orientation angle: %f", currentAang * 180 / 3.14);
		}break;
		case 7:{
			// turn towards goal? else we just set values here and this state is useless
			goal[0] = 0;
			goal[1] = 0;
			state = 0;
			
		}break;
		default:
		{
			*vel = .15;
			if(turtlebot_inputs.leftBumperPressed + turtlebot_inputs.rightBumperPressed + turtlebot_inputs.centerBumperPressed + turtlebot_inputs.sensor0State + turtlebot_inputs.sensor1State + turtlebot_inputs.sensor2State > 0)
			{
				startTime = turtlebot_inputs.nanoSecs;
				state = 1;
				if(turtlebot_inputs.leftBumperPressed > 0 ||  turtlebot_inputs.sensor0State > 0){
					bump = 1;
				}
				else{
					bump = -1;
				}
				
				
			}
			
			
		}break;
	}
	//here are the various sound value enumeration options
	//soundValue.OFF
	//soundValue.RECHARGE
	//soundValue.BUTTON
	//soundValue.ERROR
	//soundValue.CLEANINGSTART
	//soundValue.CLEANINGEND 

}

