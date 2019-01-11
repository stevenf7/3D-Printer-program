#include "PC_FileIO.c"
const float KXPROP = 0.2;//tune
const float KYPROP = 0.2;//tune
const float XTOL = 10;//tune
const float YTOL = 10;
const int CAP=8;

void flowControl (int flow){
if(flow==1){
motor[motorB]= 5;
}
else{
motor[motorB] = 0;
}
}

float dataConvX(float x_coor){//converts x y coordinate encoder counts
x_coor = 5.747 * x_coor;

return x_coor;
}

float dataConvY(float y_coor){//converts x y coordinate encoder counts
y_coor = 5.747 * y_coor;
return y_coor;
}

bool testFileOK(TFileHandle & fin, int bounds)
{
	float nextVal=0;
	float xpos=0;
	readFloatPC(fin,nextVal);
	while(readFloatPC(fin,nextVal))
	{
		xpos+=dataConvX(nextVal);
		if(fabs(xpos)>=bounds/2.0)
		{
			displayBigTextLine(4,"OUT OF BOUNDS");
			wait1Msec(1500);
			return true;
		}
		readFloatPC(fin, nextVal);
		readFloatPC(fin,nextVal);
	}

	displayBigTextLine(5,"FILE OK");
	closeFilePC(fin);
	return false;


}

void Prop (float & xOptimal,float & yOptimal, float &Xerror, float &Yerror )
{
	float angle=0.0;
		int motorSpeedA = 0;
		int motorSpeedC = 0;
		int motorSpeedD = 0;
	if ((xOptimal!=0&&yOptimal!=0)){
		angle = atan(yOptimal/xOptimal);

		angle = fabs(angle);

		Xerror = 1.0*xOptimal -nMotorEncoder[motorA];
		Yerror = 1.0*yOptimal -nMotorEncoder[motorC];

		motorSpeedA = Xerror*KXPROP*sin(angle);
		motorSpeedC = Xerror*KXPROP*sin(angle);
		motorSpeedD = -Yerror*KYPROP*cos(angle);
	if (abs(motorSpeedA)>CAP){
		motor[motorA] = CAP*sin(angle);
	}
	else{
		motor[motorA] = motorSpeedA;
	}
	if (abs(motorSpeedC)>CAP){
		motor[motorC] = CAP*cos(angle)*Yerror/abs(Yerror);
		motor[motorD] = -CAP*cos(angle)*Yerror/abs(Yerror);
	}

	else{
		motor[motorC] = Yerror*KYPROP*cos(angle);
		motor[motorD] = -Yerror*KYPROP*cos(angle);
	}

}
	else{
		Xerror = xOptimal -nMotorEncoder[motorA];
		Yerror = yOptimal -nMotorEncoder[motorC];
		motorSpeedA = Xerror*KXPROP;
		motorSpeedC = Yerror*KYPROP;
		motorSpeedD = -Yerror*KYPROP;

		if(abs(motorSpeedA)>CAP){
		motor[motorA] = CAP*motorSpeedA/abs(motorSpeedA);
}
		else{
		motor[motorA] = Xerror*KXPROP;


		if(abs(motorSpeedC)>CAP){
		motor[motorC] = CAP*motorSpeedC/abs(motorSpeedC);
		motor[motorD] = -CAP*motorSpeedC/abs(motorSpeedC);
		}
		else{
			motor[motorC] = Yerror*KYPROP;
			motor[motorD] = -Yerror*KYPROP;
		}


}
}
}

void Finish(){
displayBigTextLine(1,"Object complete");
flowControl(0) 
//move motor toSensorValue
motor[motorA] = -15;
while (SensorValue[S1]== 0&&SensorValue[S2] ==0)
{}
motor[motorA] = 0;
motor[motorC] = 50;
motor[motorD] = -50;
wait1Msec(4000);

}
int mapping (){
	motor[motorA] = -20;
	while (SensorValue[S2] ==0)
	{}
	motor[motorA] = 0;
	wait1Msec(1000);
	nMotorEncoder[motorA]= 0;


		motor[motorA] = 20;

	while (SensorValue[S1] ==0)
		{}
	motor[motorA] = 0;
	wait1Msec(1000);
	int totalDistance = 0;
	totalDistance = nMotorEncoder[motorA];
	motor[motorA]=-20;
	while(nMotorEncoder[motorA]>totalDistance/2)
	{}

	motor[motorA]=0;
	return totalDistance;

}


void DriverControl()
{
	int flow=0;
while (getButtonPress(buttonEnter) ==0){
if(Sensorvalue(S3))
{
	flow=1;
}
else
{
	flow=0;
}
if(getButtonPress(buttonLeft) ==1)
{
		motor[motorC] = CAP;
		motor[motorD] = -CAP;
		motor[motorA] = 0;
}
if(getButtonPress(buttonRight) ==1)
{
		motor[motorC] = -CAP;
		motor[motorD] = CAP;
		motor[motorA] = 0;
}
if(getButtonPress(buttonUp) ==1)
{
		motor[motorC] = 0;
		motor[motorD] = 0;
		motor[motorA] = CAP;
}
if(getButtonPress(buttonDown)==1)
{
		motor[motorC] = 0;
		motor[motorD] = 0;
		motor[motorA] = -CAP;
}


}

motor[motorC] = 50;
motor[motorD] = -50;
wait1Msec(4000);
motor[motorA] = motor[motorC] = motor[motorD] = 0;
}








task main(){
displayTextLine(1,"please select mode");
displayTextLine(2,"LEFT_BUTTON for Autonomous");
displayTextLine(3,"RIGHT_BUTTON for Driver Control");

while(getButtonPress(buttonAny)==0)
{}
eraseDisplay();
bool isButtonRight=false;
if(getButtonPress(buttonRight))
isButtonRight=true;

while(getButtonPress(buttonAny) ==1)
{}



	bool forceStop=false;

	float xOptimal = 0;
	float yOptimal = 0;
	float Xerror = 0;
	float Yerror = 0;
	int flow = 1;
//	float xporportion=0,yporportion=0;
	SensorType[S1] = sensorEV3_Touch;
	SensorType[S2] = sensorEV3_Touch;
	SensorType[S3] = sensorEV3_Touch;

	TFileHandle fin;
	TFileHandle filein;
	//bool fileOK=openReadPC(fin,"squarecoordinates.txt");


	int maxVal=mapping();
	if(isButtonRight){
	DriverControl();
	}
	else
	{
	displayBigTextLine(0,"Press to Start -->");
	while (SensorValue[S3]==0)
	{}
	while (SensorValue[S3]!=0)
	{}
	displayBigTextLine(0,"Press to");
	displayBigTextLine(2,"TERMINATE-->");
	nMotorEncoder[motorA] = 0;
	nMotorEncoder[motorC] = 0;


	bool fileOK=openReadPC(fin,"Square.txt");
		if (!fileOK)
	{
		displayBigTextLine(0,"File Fail");
		wait1Msec(10000);
	}

	forceStop=testFileOK(fin,maxVal);

	wait1Msec(1000);
	fileOK=openReadPC(filein,"Square.txt");//kinda sketchy
time1[T1] = 0;
	while (readIntPC(filein,flow)){
	if(!forceStop){
		readFloatPC(filein,xOptimal);
		readFloatPC(filein,yOptimal);
		flowControl(flow);
		xOptimal = dataConvX(xOptimal);
		yOptimal = dataConvY(yOptimal);

		do{
		if (SensorValue[S3]||SensorValue[S2]||SensorValue[S1])
			forceStop=true;
		Prop (xOptimal,yOptimal,Xerror,Yerror);
		wait1Msec(20);
		}while ((fabs(Xerror)> XTOL || fabs(Yerror)> YTOL)&&!forceStop);

		nMotorEncoder[motorA] = 0;
		nMotorEncoder[motorC] = 0;
		wait1Msec(100);
		motor[motorA]=motor[motorC]=motor[motorD]=0;


		wait1Msec(50); //to not hoard CPU
		}
	}
	if (!forceStop){
	flow=0;
	flowControl(flow);
	Finish();
	}
	closeFilePC(filein);
}
int time = time1[T1];
time = time / 1000;
displayTextLine(5, "print time: %d seconds",time);
wait1Msec(5000);
	}

