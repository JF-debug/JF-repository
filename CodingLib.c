#pragma config(Sensor, port8,  Gyro,            sensorVexIQ_Gyro)
#pragma config(Motor,  motor8,          LeftMotor,     tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor12,          RightMotor,    tmotorVexIQ, PIDControl, encoder)

void StraightDistance(float BaseSpeed, float Kp, int heading, float distanceCm, float SlowDownZone, float HoldZone, float EndSpeed){
	setMotorBrakeMode(motor8, motorHold);
	setMotorBrakeMode(motor12, motorHold);
	resetGyro(port8);
	resetMotorEncoder(LeftMotor);
	resetMotorEncoder(RightMotor);
	setMotorEncoderUnits(encoderDegrees);
	distanceCm = distanceCm/360*20*3;
	SlowDownZone = SlowDownZone/360*20*3;
	HoldZone = HoldZone/360*20*3;
		while(true){
			int error = getGyroDegrees(port8)-heading;
			float MotorInput = error*Kp;
			float dis = getMotorEncoder(LeftMotor)/360*20*3+getMotorEncoder(RightMotor)/360*20*3/2;
			int TheSpeedLeft = BaseSpeed+MotorInput;
			int TheSpeedRight = BaseSpeed-MotorInput;
				if(dis<(distanceCm-(SlowDownZone+HoldZone))){
					setMotorSpeed (LeftMotor, TheSpeedLeft);
					setMotorSpeed (RightMotor, TheSpeedRight);
				}else{
					if(dis>HoldZone){
						//http://www.mathcentre.ac.uk/resources/uploaded/mc-ty-strtlines-2009-1.pdf
						int Y1 = BaseSpeed;
						int Y2 = EndSpeed;
						int X1 = distanceCm-HoldZone-SlowDownZone;
						int X2 = distanceCm-HoldZone;
						int m = (Y2-Y1)/(X2-X1);
						int mX = m*dis;
                        int c = EndSpeed;
                        int y = m*dis+c;
						setMotorSpeed (LeftMotor, TheSpeedLeft+y);
						setMotorSpeed (RightMotor, TheSpeedRight-y);
					}else{
					setMotorSpeed (LeftMotor, BaseSpeed+EndSpeed);
					setMotorSpeed (RightMotor, BaseSpeed-EndSpeed);
					}
				}
			}

			if ( dis>= distanceCm ) {
				setMotorSpeed(LeftMotor, 0);
				setMotorSpeed(RightMotor, 0);
				break;
			}
		}
task main(){
	StraightDistance(10, 0.4 ,0,100, 50, 30, 10);
}
