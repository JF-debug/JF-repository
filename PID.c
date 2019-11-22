#pragma config(Sensor, port8,  LED,            sensorVexIQ_LED)
#pragma config(Sensor, port9,  Gyro,           sensorVexIQ_Gyro)
#pragma config(Motor,  motor1,          LeftMotor,     tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor6,          RightMotor,    tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor7,          Claw,          tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         RearArm,       tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor12,         Arm,           tmotorVexIQ, PIDControl, reversed, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

void StraightDistance(float BaseSpeed, float Kp, int heading, float distanceCm, /*float SlowDownZone,*/ float HoldZone, float EndSpeed, float GearRatio){
	setMotorBrakeMode(motor1, motorHold);
	setMotorBrakeMode(motor6, motorHold);
	resetGyro(port9);
	resetMotorEncoder(LeftMotor);
	resetMotorEncoder(RightMotor);
	setMotorEncoderUnits(encoderDegrees);
	distanceCm = distanceCm/360*20*GearRatio;
	HoldZone = HoldZone/360*20*GearRatio;

	while(true){
		int error = getGyroDegrees(port9)-heading;
		float MotorInput = error*Kp;
		float dis = ((((getMotorEncoder(LeftMotor)/360)*20)*GearRatio)+(((getMotorEncoder(RightMotor)/360)*20)*GearRatio))/2;
		int TheSpeedLeft = BaseSpeed+MotorInput;
		int TheSpeedRight = BaseSpeed-MotorInput;
		int FESL = EndSpeed+MotorInput;
		int FESR = EndSpeed-MotorInput;

		if(distanceCm>HoldZone){
			if(dis<(distanceCm-HoldZone)){
				setMotorSpeed (LeftMotor, TheSpeedLeft);
				setMotorSpeed (RightMotor, TheSpeedRight);
			}else{
				if(dis>HoldZone){
					//http://www.mathcentre.ac.uk/resources/uploaded/mc-ty-strtlines-2009-1.pdf
					//int Y1 = BaseSpeed;
					//int Y2 = EndSpeed;
					//int X1 = distanceCm-HoldZone-SlowDownZone;
					//int X2 = distanceCm-HoldZone;
					//int m = (Y1-Y2)/(X1-X2);
					//int mX = m*dis;
					//setMotorSpeed (LeftMotor, TheSpeedLeft+mX);
					//setMotorSpeed (RightMotor, TheSpeedRight-mX);
				setMotorSpeed(LeftMotor,TheSpeedLeft);
				setMotorSpeed(RightMotor, TheSpeedRight);
				}else{
				setMotorSpeed (LeftMotor, FESL);
				setMotorSpeed (RightMotor, FESR);
				}
			}
		}
		else{
	 		if(BaseSpeed>20){
				setMotorSpeed (LeftMotor, (20)+MotorInput);
				setMotorSpeed (RightMotor, (20)-MotorInput);
			}else{
				setMotorSpeed(LeftMotor, BaseSpeed+MotorInput);
				setMotorSpeed(RightMotor, BaseSpeed-MotorInput);
			}
		}

		if ( dis>= distanceCm ) {
			setMotorSpeed(LeftMotor, 0);
			setMotorSpeed(RightMotor, 0);
			break;
		}
	}
}


void  ArmRaise(int Level, int Speed){
	int iArmLv[4] = {20, 150, 385, 545};

	if(getMotorEncoder(Arm)>iArmLv[Level]){
		while(true){
			setMotorSpeed(Arm, -Speed);
			if(getMotorEncoder(Arm)<=iArmLv[Level]){
				setMotorSpeed(Arm, 0);
				break;
			}
		}
	}
	else if( getMotorEncoder(Arm)<iArmLv[Level]){
		while(true){
			setMotorSpeed(Arm, Speed);
			if(getMotorEncoder(Arm)>=iArmLv[Level]){
				setMotorSpeed(Arm, 0);
				break;
			}
		}
	}
	else{
		return;
	}
}

void toggleclaw(){
	if(getMotorEncoder(Claw)>20){
		setMotorSpeed(Claw, -100);
			while(true){
				if(getMotorEncoder(Claw)<10){
					break;
				}
			}
		}else{
		setMotorSpeed(Claw, 100);
		while(true){
		if(getMotorEncoder(Claw)>80){
			break;
		}
	}
	}
	}

	void waitLED(){
		while(true){
			if(getTouchLEDValue(port8)==1){
				break;
				return;
			}
		}
	}

	void killallmotors(){
	setMotorSpeed(Arm, 0);
	setMotorSpeed(LeftMotor, 0);
	setMotorSpeed(RightMotor, 0);
	setMotorSpeed(Claw, 0);
	setMotorSpeed(RearArm, 0);
	}
task main(){
	setMotorSpeed(Claw, 100);
	setMotorSpeed(RearArm, 100);
	setMotorSpeed(Arm, -100);
	wait1Msec(2000);
	resetMotorEncoder(Claw);
	resetMotorEncoder(RearArm);
	resetMotorEncoder(Arm);
	killallmotors();
	waitLED();
	//BaseSpeed, Kp, heading, distanceCm, /*float SlowDownZone,*/ float HoldZone, float EndSpeed, float GearRatio
	StraightDistance(50, 0.4 ,0,100, 30, 10, 1.5);
	ArmRaise(3, 50);
	toggleclaw();
	wait1Msec(300000000000);
}
