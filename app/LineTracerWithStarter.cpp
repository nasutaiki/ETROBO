/******************************************************************************
 *  LineTracerWithStarter.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/26
 *  Implementation of the Class LineTracerWithStarter
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "LineTracerWithStarter.h"
#include "ev3api.h"
#include "SonarSensor.h"




/**
 * コンストラクタ
 * @param lineTracer ライントレーサ
 * @param starter    スタータ
 */
LineTracerWithStarter::LineTracerWithStarter(LineTracer* lineTracer,
											const Starter* starter,
											Calibration* calibration,
											LineMonitor* lineMonitor,
											ev3api::Motor& tailWheel,
											ev3api::SonarSensor& sonarSensor,
											ev3api::Motor& leftWheel,
											ev3api::Motor& rightWheel,
											char* bt_data)
	: mLineTracer(lineTracer),
	  mStarter(starter),
	  mCalibration(calibration),
	  mLineMonitor(lineMonitor),
	  mTailWheel(tailWheel),
	  mSonarSensor(sonarSensor),
	  mLeftWheel(leftWheel),
	  mRightWheel(rightWheel),
	  mState(UNDEFINED){
		mBt_data = bt_data;
}

/**
 * デストラクタ
 */
LineTracerWithStarter::~LineTracerWithStarter() {
}

/**
 * ライントレースする
 */
void LineTracerWithStarter::run() {
    switch (mState) {
    case UNDEFINED:
        execUndefined();
        break;
    case CALIBRATION_GYRO:
        execCalibrationGyro();
        break;
    case CALIBRATION_BLACK:
        execCalibrationBlack();
        break;
    case CALIBRATION_WHITE:
        execCalibrationWhite();
        break;
    case CALIBRATION_GRAY:
        execCalibrationGray();
        break;
    case WAITING_FOR_START:
        execWaitingForStart();
        break;
    case WALKING:
        execWalking();
        break;
    case FIGURE:
        execFigure();
        break;
    case LAST:
        execLast();
        break;
	default:
        break;
    }
}

/**
 * 未定義状態の処理
 */
void LineTracerWithStarter::execUndefined() {
	mTailWheel.reset();
	mTailWheel.setCount(0);
	tailControll(85); //テイル降ろす
	//tailControll2(80);
	mCalibration->init();
	mState = CALIBRATION_GYRO;
}

/**
 * ジャイロセンサのキャリブレーション
 */
void LineTracerWithStarter::execCalibrationGyro() {
//	if (mCalibration->calibrateGyro(mStarter->isPushed()) == true) {
	if (mCalibration->calibrateGyro(1) == true) {
		mState = CALIBRATION_BLACK;
	}
}

/**
 * 黒キャリブレーション
 */
void LineTracerWithStarter::execCalibrationBlack() {
//	if (mCalibration->calibrateBlack(mStarter->isPushed()) == true) {
	if (mCalibration->calibrateBlack(1) == true) {
		mState = CALIBRATION_WHITE;
	}
}

/**
 * 白キャリブレーション
 */
void LineTracerWithStarter::execCalibrationWhite() {
	if (mCalibration->calibrateWhite(mStarter->isPushed()) == true) {
		mState = CALIBRATION_GRAY;
	}
}
/**
 * 灰キャリブレーション
 */
void LineTracerWithStarter::execCalibrationGray() {
	if( mCalibration->calibrationColor(mStarter->isPushed()) == true ){
		while(mStarter->isPushed());
		mState = WAITING_FOR_START;
	}
}
/**
 * 開始待ち状態の処理
 */
void LineTracerWithStarter::execWaitingForStart() {
	if (mStarter->isPushed() || *mBt_data == '1') {//ボタン押すＯＲブルーツースから１送るとスタート
		mCalibration->calibrateLineThreshold();
		mState = WALKING;
	}
}

/**
 * 走行中状態の処理
 */
void LineTracerWithStarter::execWalking() {
	static int count = 0;
	if(mTailWheel.getCount() < 1) mTailWheel.setPWM(0);
	switch(count){
		case 0:									// テイル回収
			mLineTracer->runLOW();
			if(mLeftWheel.getCount() > 3){
				mTailWheel.setPWM(-48);
				count++;
			}
			break;
		case 1:									// テイル回収したあと走行する
			mLineTracer->runLOW();
			if(mTailWheel.getCount() == 0) mTailWheel.setPWM(0);
			if(mLeftWheel.getCount() > 100){
				count=2;
				//mState = FIGURE;
			}
			break;
		case 2:									// 第一直線
			mLineTracer->run90();
			if(mLeftWheel.getCount() > 2000){
				count=3;
			}
			break;
		case 3:									// 第1カーブ
			mLineTracer->run80();
			if(mLeftWheel.getCount() > 4800){
				count++;
			}
			break;
		case 4:									// 第2カーブ
			mLineTracer->runLOW();
			if(mLeftWheel.getCount() > 6500){
				count++;
			}
			break;
		case 5:									// 第3カーブ
			mLineTracer->runLOW();
			if(mLeftWheel.getCount() > 12200){
				count++;
			}
			break;
		case 6:									// 第2直線
			mLineTracer->run90();
			if(mLeftWheel.getCount() > 14600){
				count++;
			}
			break;
		case 7:									// フィギュア前まで
			mLineTracer->run3();
			if(mLeftWheel.getCount() > 15850){  //10000
				mState = FIGURE;
				//mLeftWheel.setCount(0);
			}
			break;
	}
}

/**
 * 難所１の処理
 */
void LineTracerWithStarter::execFigure() {
	static int8_t countF = 0;
	static int32_t time = 0;
	int sonarInt = 0;
	switch(countF){
		case 0:
			mLineTracer->run3();
			sonarInt = mSonarSensor.getDistance();		//超音波センサから値を取る
			if(sonarInt <= 20){
				countF = 1;
				mTailWheel.setPWM(10);
			}
			break;
		case 1:
			mLineTracer->runStop();						// その場で止まる
			if(mTailWheel.getCount() >= 75){			// テイル降ろす
				mTailWheel.setPWM(0);
				mTailWheel.setCount(75);
				if(mLineTracer->getAngle() == false) {
					countF = 2;
				}
			}
			break;
		case 2:
			mLeftWheel.setPWM(10);
			mRightWheel.setPWM(10);
			time++;
			if(time > 120) {
				time = 0;
				mTailWheel.setPWM(-1);
				countF = 3;
			}
			break;
		case 3:
			mLeftWheel.setPWM(0);
			mRightWheel.setPWM(0);
			if(mTailWheel.getCount() == 65){			// テイルおろし終わったら
				mTailWheel.setPWM(0);
			}
			time++;
			if(time > 500){
				time = 0;
				countF = 4;
				mLeftWheel.setCount(0);
			}
			break;
		case 4:
			mLineTracer->runOnOff();			
			if(mLeftWheel.getCount() >500){
				countF = 5;
			}
			break;
		case 5:
			mLineTracer->runBack2();
			time++;
			if(time > 2000){
				time = 0;
				countF = 6;
			}
			break;
		case 6:
			mLineTracer->runBack1();
			time++;
			if(time > 2000){
				time = 0;
				countF = 7;
				mLeftWheel.setCount(0);
			}
			break;
		case 7:
			mLineTracer->runOnOff();
			if(mLeftWheel.getCount() > 700){
				countF = 8;
			}
			break;
		case 8:
			mLineTracer->runBack2();
			time++;
			if(time >2000){
				time = 0;
				countF = 9;
			}
			break;
		case 9:
			mLineTracer->runBack1();
			time++;

			if(time >2000){
				time = 0;
				countF = 10;
				mLeftWheel.setCount(0);
			}
		break;
		case 10:
			mLineTracer->runOnOff();
			if(mLeftWheel.getCount() > 800){
				countF = 11;
			}
		break;
		case 11:
			mLeftWheel.setPWM(-10);
			mRightWheel.setPWM(-10);
			mTailWheel.setPWM(3);
			if(mTailWheel.getCount()>80){
				mLeftWheel.setPWM(0);
				mRightWheel.setPWM(0);
				mTailWheel.setPWM(0);
				mLeftWheel.setCount(0);
				time++;
			}
			time++;
			if(time > 450){
				mTailWheel.setPWM(0);
				time = 0;
				countF = 12;
				mState = LAST;
				sonarInt = 0;
			}
		break;
	}
}
/**
 * 難所２の処理
 */
void LineTracerWithStarter::execLast(){
	static int8_t countL = 0;
	static int32_t time = 0;
	switch(countL){
		case 0:
			mLineTracer->runOnOff2();
		if(mLineTracer->runOnOff2() == 1){
			mLeftWheel.setPWM(0);
			mRightWheel.setPWM(0);
			mLeftWheel.setCount(0);
			countL++;
		}
		break;
		case 1:
			mLeftWheel.setPWM(0);
			mRightWheel.setPWM(0);
			time++;
			if(time >1000){
				time = 0;
				countL++;
				mLeftWheel.setCount(0);
			}
		break;
		case 2:
		mLineTracer->runOnOff3();
		if(mLineTracer->runOnOff3() == 3){
			mLeftWheel.setPWM(0);
			mRightWheel.setPWM(0);
			mLeftWheel.setCount(0);
			countL++;
		}
		break;
		case 3:
			mLeftWheel.setPWM(0);
			mRightWheel.setPWM(0);
			time++;
			if(time >1000){
				time = 0;
				countL++;
				mLeftWheel.setCount(0);
			}
		break;
		case 4:
		mLineTracer->runOnOff2();
		if(mLeftWheel.getCount() > 150){
			countL++;
		}
		break;
	case 5:
			mLineTracer->runOnOff();
			mLeftWheel.setPWM(0);
			mRightWheel.setPWM(0);
		break;
	}


}
void LineTracerWithStarter::tailControll(int deg) {
	if(deg>10)mTailWheel.setPWM(48);
	else if(deg == 0)mTailWheel.setPWM(-48);
	while(1){
		if(mTailWheel.getCount() > deg)break;
	}
	mTailWheel.setPWM(0);
}
void LineTracerWithStarter::tailControll2(int deg) {
	if(deg == 80)mTailWheel.setPWM(30);
	else if(deg == 0)mTailWheel.setPWM(-30);
	while(1){
		if(mTailWheel.getCount() > deg)break;
	}
	mTailWheel.setPWM(0);
}
