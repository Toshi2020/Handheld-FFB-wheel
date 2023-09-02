/*****************************************************************************
*
*	MyAFFBWheel.ino -- FFBステアリング(Arduino Pro Micro専用)
*
*	ビルドの方法
*	・こちらのAFFBWheel↓の、
*	　https://github.com/vsulako/AFFBWheel
*	　AFFBWheelフォルダから、
*	　以下のファイル以外のファイルをこのフォルダにコピーする
*		AFFBWheel.ino, bb_i2c.cpp, bb_i2c.h, motor.cpp, motor.h,
*		multiturn.cpp, multiturn.h, 
*	・Arduino IDE 1.8.19以上でボードタイプはArduino Leonardoを選択してビルド
*
*	rev1.0	2023/08/20	initial revision by	Toshi
*
*****************************************************************************/

#include "wheel.h"
#include "settings.h"

// Arduino ピン設定
#define PIN_AY A2		// Joystick Y
#define PIN_SW2 A1		// JoystickのプッシュSW
#define PIN_STR A0		// ステア
#define PIN_SW3 4		// タクトSW3
#define PIN_SW4 5		// タクトSW4
#define PIN_SW1 8		// タクトSW1
#define PIN_CW 9		// モーター正転
#define PIN_CCW 10		// モーター逆転

Wheel_ wheel;			// ステアリングホイールオブジェクト
SettingsData settings;	// セッティングデータ構造体
bool fReqCenter;		// ステアリングをセンターに戻す要求
unsigned long LastTime;	// ループ時間計測用
int bug;
bool fEnduranceTest, fForceTest;
int RxForce;			// 受信したフォース
int Fmax = 255;			// 最大フォース

// プロトタイプ宣言
void PwmForce(int force, int ad_steer, int velocity);
int EnduranceTestForce(int ad_steer);
int MoveForce(bool* frun, int target, int ad_steer);
long map2(long x, long in_min, long in_max, long out_min, long out_max);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void Filter(float* filt, float dat, float fact);
void AddOnTime(bool flag, int* ontime);
/*----------------------------------------------------------------------------
	セットアップ
----------------------------------------------------------------------------*/
void setup()
{
	// ピンモードの設定
	pinMode(PIN_SW1, INPUT_PULLUP);
	pinMode(PIN_SW2, INPUT_PULLUP);
	pinMode(PIN_SW3, INPUT_PULLUP);
	pinMode(PIN_SW4, INPUT_PULLUP);
	pinMode(PIN_CW, OUTPUT);
	pinMode(PIN_CCW, OUTPUT);

	// Timer1のPWM周波数を490Hz→31.25kHzに変更(HブリッジBD6211Fは20kHz以上要)
	TCCR1B &= B11111000;
	TCCR1B |= B00000001;

	Serial.begin(115200);	// デバッグ用ハードウェアシリアル

	// FFBセッティング
	FfbSetting();

	// テストモードか？
	fEnduranceTest = !digitalRead(PIN_SW1) && !digitalRead(PIN_SW3) &&
					 !digitalRead(PIN_SW4);
	fForceTest = !digitalRead(PIN_SW3) && !digitalRead(PIN_SW4);

	if (!fEnduranceTest && !fForceTest)	// 通常モードなら
	{
		// ステアリングをセンターに戻す要求を出す
		MoveForce(&fReqCenter, 512, 512);	// 初期化
		fReqCenter = true;
		Fmax = 180;		// PWM最大指示値
						// (USB2.0の規格では最大電流が0.5[A]なので、
						// ストール電流を0.3[A]程度にしておくのが良いと思う)
						// USBモニタによる実測
						// 255 = 0.52[A]
						// 200 = 0.49[A]
						// 180 = 0.33[A]
	}

	LastTime = millis();
}
/*----------------------------------------------------------------------------
	メインループ
----------------------------------------------------------------------------*/
#define ST_MAX 10230// ステアリング最大指示値
					// ↑これでおおむねint16のフルスケールになるようだ
#define FILTMIN 0.5	// ステアフィルタ定数(一番軽い時)
#define FILTMAX 0.01// ステアフィルタ定数(一番重い時)
#define SDELTA 500	// ステアフィルタが一番軽くなる時のステア速度
#define ACMAX 1000	// アクセルA/D最大値
#define ACMIN 510	// アクセルA/D最小値
#define BKMAX 490	// ブレーキA/D最大値
#define BKMIN 0		// ブレーキA/D最小値
#define DAMPC 0.2	// ステアダンピング係数(大きくするとダンピング大)
#define DPV 10		// ステアダンピングを効かせる最低回転速度
#define CDELTA 204	// 100ms当たりのクラッチ戻し量
#define FDEADMIN 30	// モーターが回らないPWM値
void loop()
{
	bool fsw1, fsw2, fsw3, fsw4;
	int ad_y, ad_steer;
	int accel, brake, steer;
	int ffb, force, velocity;
	static float steer_filt;
	static uint32_t switchs, switchsz;
	static int dcount, clutch;
	static float filtc = FILTMIN;
	float damp;

	// A/Dを読む
	ad_y = analogRead(PIN_AY);		// Joystick Y
	ad_steer = analogRead(PIN_STR);	// ステア

	// SWを読む
	fsw1 = !digitalRead(PIN_SW1);
	fsw2 = !digitalRead(PIN_SW2);
	fsw3 = !digitalRead(PIN_SW3);
	fsw4 = !digitalRead(PIN_SW4);
	switchs = fsw4 << 3 | fsw3 << 2 | fsw2 << 1 | fsw1;	// SW情報を束ねる

	// ステアのA/D値をを100倍した値をフィルタリング
	// (10bitのA/Dでは分解能が不足なので疑似的にビットを増やす意味もある)
	Filter(&steer_filt, (float)ad_steer * 100.0, filtc);

	// アクセル、ブレーキ、ステア量を変換する
	accel = map(ad_y, ACMIN, ACMAX, 0, 1023);
	brake = map(ad_y, BKMAX, BKMIN, 0, 1023);
	steer = map((long)steer_filt, 0, 102300, -ST_MAX, ST_MAX);

	// ホイールにデータを渡す
	wheel.axisWheel->setValue(steer);
	wheel.analogAxes[AXIS_ACC]->setValue(accel);
	// SW1をクラッチに割り当てる前提
	if (fsw1)
	{
		clutch = 1023;	// SWオンで目いっぱい踏んでいることにする
	}
	else
	{
		// クラッチを踏んでいるときはブレーキホールドとする
		// すなわちブレーキを更新はクラッチオフの時のみとする
		// これはヒール＆トゥを実現するため(使わないかも)
		wheel.analogAxes[AXIS_BRAKE]->setValue(brake);
	}
	// クラッチはアナログ軸にも同時に出力する
	// ゲーム側でSWとどちらでも選択可能にするため
	wheel.analogAxes[AXIS_CLUTCH]->setValue(clutch);

	// SWのチャタリング除去(デバウンス処理)
	if (switchsz != switchs)	// SWに変化があったら
	{
		dcount = 0;				// カウンタ初期化
		switchsz = switchs;		// 変化した時のSWの状態を保持
	}
	else
	{
		if (dcount < settings.debounce)	// 規定時間に達するまでは
		{
			dcount++;	// カウンタを+1するだけでSW情報を伝えない
		}
		else
		{
			wheel.buttons = switchs;	// ホイールにSW情報を伝える
		}
	}
	// ホイール指示アップデート
	wheel.update();
	velocity = wheel.axisWheel->velocity;	// ステア回転速度を取得

	// ここで次回のためにステアのフィルタ定数を決定しておく
	// (フィルタはステア速度が遅い時には重く、速い時には軽くする)
	filtc = mapf((float)abs(velocity), 0, SDELTA, FILTMAX, FILTMIN);

	if (fEnduranceTest)	// 耐久試験モードなら
	{
		force = EnduranceTestForce(ad_steer);	// 耐久試験用のフォースを得る
	}
	else if (fForceTest)	// フォーステストモードなら
	{
		RxData();			// データ受信
		force = RxForce;	// 受信したフォースを出力
	}
	else
	{
		if (fReqCenter)	// センタリング要求中なら
		{
			force = MoveForce(&fReqCenter, 512, ad_steer);	// センタリング
		}
		else	// 通常動作中なら
		{
			// ホイールからFFBデータを受ける
			ffb = wheel.ffbEngine.calculateForce(wheel.axisWheel);

			// モーターによるステア回転が速すぎるのでダンピングする
			damp = (float)abs(velocity) * DAMPC;// 速度の係数倍を分母に
			if (damp != 0.0)	// 0割防止
			{
				damp = 100.0 / damp;	// 速度に反比例
				damp = min(1.0, damp);	// 0～1.0 1.0ならダンピング効果なし
				// FFBによりステアが回転していたら
				if ((velocity <= -DPV && ffb > 0) ||
					(velocity >= DPV && ffb < 0))
				{
					// 速度に反比例してFFBを減少
					ffb = (int)((float)ffb * damp);
				}
			}
			// モーターの範囲内に変換する
			force = map(ffb, -16383, 16383,
							-Fmax + FDEADMIN, Fmax - FDEADMIN);
		}
	}
	// フォースをPWM出力
	PwmForce(force, ad_steer, velocity);

	// 定時処理
	if (millis() - LastTime >= 100L)	// 100[ms]
	{
		LastTime = millis();

		if (clutch)
		{
			clutch -= CDELTA;			// クラッチを徐々に離す
			clutch = max(0, clutch);	// 0以上
		}

		// ↓デバッグ時にコメントを外す
//		Serial.println(ad_y);
//		Serial.println(ad_steer);
//		Serial.println(steer);
//		Serial.println(wheel.axisWheel->velocity);
//		Serial.println(damp);
//		Serial.println(ffb);
//		Serial.println(force);
//		Serial.println(bug);
	}
}
/*----------------------------------------------------------------------------
	耐久試験(左右にステップ駆動)用のフォースを得る
	書式 int ret = EnduranceTestForce(int ad_steer);

	int ret;		フォース -Fmax～+Fmax
	int ad_steer;	ステアA/D値 0～1023
----------------------------------------------------------------------------*/
#define STEPDELTA 5	// A/D幅
int EnduranceTestForce(int ad_steer)
{
	static int steps = STEPDELTA, force = 255;
	static bool fup = true;
	int upper, lower;
	static long bcount;

	upper = 512 + steps;
	lower = 512 - steps;
	if (fup && ad_steer >= upper)	// ステアが右回転して位置に達した？
	{
		force = Fmax;	// ステア左回転に転じる
		fup = false;
	}
	else if (!fup && ad_steer <= lower)	// ステアが左回転して位置に達した？
	{
		force = -Fmax;	// ステア右回転に転じる
		fup = true;
		steps += STEPDELTA;
		if (steps >= 512)	// 最後まで行った？
		{
			steps = STEPDELTA;// 最初に戻る
			bcount++;
			Serial.println(bcount);	// 回数を表示
		}
	}
	return force;
}
/*----------------------------------------------------------------------------
	位置制御用のフォースを得る
	書式 ret = MoveForce(bool* frun, int target, int ad_steer);

	int ret;		フォース -255～+255 正の時ステアが左に回転する
	bool* frun;		作動要求フラグ
	int target;		目標A/D位置0～1023
	int ad_steer;	ステアA/D値 0～1023
----------------------------------------------------------------------------*/
#define PGAIN 0.3	// 比例ゲイン
#define IGAIN 0.1	// 積分ゲイン
#define ERRRANGE 20	// 終了判定A/D範囲
#define ENDTIME 50	// 終了判定時間5[s]x10
#define FSTIME 100	// 強制終了時間10[s]x10
int MoveForce(bool* frun, int target, int ad_steer)
{
	static float err, i;
	static int force, runtime, endtime;
	static long lasttime;

	if (!*frun)	// 初期化要求？
	{
		i = 0.0;
		force = runtime = endtime = 0;
		lasttime = millis();
		return 0;
	}
	if (millis() - lasttime >= 100L)	// 100ms
	{
		lasttime = millis();
		AddOnTime(*frun, &runtime);			// 稼働時間

		err = (float)(ad_steer - target);	// 位置偏差
		if (abs(force) < Fmax)				// サチってなければ
		{
			if (runtime >= 10)	// 最初の1秒はI項の累積しない
			{
				i += (float)err / 10.0;			// I項累積
			}
		}
		// PI
		force = (int)(err * PGAIN + i * IGAIN);
		force = constrain(force, -Fmax, Fmax);

		// エラーが範囲内に入っている時間
		AddOnTime(abs(err) < ERRRANGE, &endtime);
		// 強制終了時間を経過したか、偏差が範囲内に入っている時間が規定以上？
		if (runtime >= FSTIME || endtime >= ENDTIME)
		{
			*frun = false;		// 制御終了
			i = 0.0;
			force = runtime = endtime = 0;
			if (target == 512)	// センターへの指示だった場合
			{
				wheel.axisWheel->center();	// センターに戻ったことを伝える
			}
		}
//		Serial.print(err);
//		Serial.print(", ");
//		Serial.print(i);
//		Serial.print(", ");
//		Serial.println(force);
	}
	return force;
}
/*----------------------------------------------------------------------------
	フォースをPWM出力
	書式 void PwmForce(int force, int ad_steer, int velocity);

	int force;		フォース -Fmax～+Fmax 正の時ステアが左に回転する
	int ad_steer;	ステアA/D値 0～1023
	int velocity;	ステア速度
----------------------------------------------------------------------------*/
#define FDEADMAX 60	// モーターが回り始めるPWM値
#define FDELTA 0.1	// オフセット増加幅
#define FDEADV 30	// 回転とみなす速度
void PwmForce(int force, int ad_steer, int velocity)
{
	static float offset;
	static bool fbegin;
	static int forcez;

	//*** デッドゾーン除去 ***
	// (回転が始まらない限りオフセットをFDEADMINからFDEADMAXまで増やす)

	// 指示フォースが0以下から正に変化した？
	if (forcez <= 0 && force > 0)
	{
		offset = FDEADMIN;	// 初期オフセットを設定
		fbegin = true;
	}
	// 指示フォースが0以上から負に変化した？
	else if (forcez >= 0 && force < 0)
	{
		offset = -FDEADMIN;	// 初期オフセットを設定
		fbegin = true;
	}
	// 指示フォースが0？
	else if (force == 0)
	{
		offset = 0.0;	// オフセットなし
		fbegin = false;
	}
	forcez = force;
	// フォースが正
	if (fbegin && offset > 0)
	{
		// ステアが左回転したかオフセットが十分大きくなった？
		if (velocity < -FDEADV || offset >= FDEADMAX)
		{
			fbegin = false;	// これ以上は増やさない
		}
		else	// ステアが左回転していないなら
		{
			offset += FDELTA;	// オフセット量を少し増やす
		}
	}
	// フォースが負
	else if (fbegin && offset < 0)
	{
		// ステアが右回転したかオフセットが十分小さくなった？
		if (velocity > FDEADV || offset <= -FDEADMAX)
		{
			fbegin = false;	// これ以上は減らさない
		}
		else	// ステアが右回転していないなら
		{
			offset -= FDELTA;	// オフセット量を少し減らす
		}
	}
	force += (int)offset;	// デッドゾーン分を上乗せ
	force = constrain(force, -Fmax, Fmax);
	if (force > 0)
	{
		// モーター右回転→ステア左回転
		digitalWrite(PIN_CCW, LOW);
		if (ad_steer > 2)				// 左に回り切っていないなら
		{
			analogWrite(PIN_CW, force);	// ステア左回転
		}
		else
		{
			digitalWrite(PIN_CW, LOW);	// フェイルセーフで停止
		}
	}
	else if (force < 0)
	{
		// モーター左回転→ステア右回転
		digitalWrite(PIN_CW, LOW);
		if (ad_steer < 1021)			// 右に回り切っていないなら
		{
			analogWrite(PIN_CCW, -force);// ステア右回転
		}
		else
		{
			digitalWrite(PIN_CCW, LOW);	// フェイルセーフで停止
		}
	}
	else
	{
		// ブレーキ
		digitalWrite(PIN_CW, HIGH);
		digitalWrite(PIN_CCW, HIGH);
	}
}
/*----------------------------------------------------------------------------
	データ受信
----------------------------------------------------------------------------*/
#define BUFMAX 64	// 受信バッファサイズ
void RxData()
{
	char c, *p;
	static char rxbuff[BUFMAX];	// 受信バッファ
	static char pbuff;			// バッファポインタ
	int count, i;
	int rxdata;

	while (Serial.available() > 0)	// 文字を受信していれば
	{
		c = Serial.read();			// 1文字受信
		if (pbuff < BUFMAX - 2)		// バッファに余裕があるなら
		{
			rxbuff[pbuff++] = c;	// 格納
			if (c == '\n' && pbuff > 1)	// 行末か？
			{
				rxbuff[pbuff - 1] = '\0';// 文字列をターミネート
				pbuff = 0;			// ポインタを先頭に
				Serial.println(rxbuff);
				RxForce = atoi(rxbuff);	// 文字→数値変換
			}
		}
		else	// バッファフルなら
		{
			pbuff = 0;	// ポインタを先頭に
		}
	}
}
/*----------------------------------------------------------------------------
	最大最少リミット付きmap関数()
----------------------------------------------------------------------------*/
long map2(long x, long in_min, long in_max, long out_min, long out_max)
{
	long ret;

	ret = map(x, in_min, in_max, out_min, out_max);
	ret = constrain(ret, out_min, out_max);
	return ret;
}
/*----------------------------------------------------------------------------
	最大最少リミット付きmap関数()float版
----------------------------------------------------------------------------*/
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
	float ret;

	ret = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	ret = constrain(ret, out_min, out_max);
	return ret;
}
/*----------------------------------------------------------------------------
	フィルタ
	書式 void Filter(float* filt, float dat, float fact);

	float filt;		入力(1サンプル前)→出力
	float dat;		入力(今回)
	float fact;		フィルタ定数
----------------------------------------------------------------------------*/
void Filter(float* filt, float dat, float fact)
{
	*filt = (1.0 - fact) * *filt + fact * dat;
}
/*----------------------------------------------------------------------------
	フラグのオン時間の累積
	書式 void AddOnTime(bool flag, int* ontime)

	bool flag;		フラグ
	int* ontime;	オン時間
----------------------------------------------------------------------------*/
#define	TIMEMAX 30000
void AddOnTime(bool flag, int* ontime)
{
	if (flag)							// オンしてるなら
	{
		if (*ontime < TIMEMAX)
		{
			(*ontime)++;				// オン時間++
		}
	}
	else
	{
		*ontime = 0;
	}
}
/*----------------------------------------------------------------------------
	FFBセッティング(オリジナルではEEPから読み出すがここでは直接セット)
----------------------------------------------------------------------------*/
void FfbSetting()
{
	uint8_t i;

	// 各ゲイン 1024で100[%]
	settings.gain[GAIN_TOTAL] = 1024;
	settings.gain[GAIN_CONSTANT] = 1024;
	settings.gain[GAIN_RAMP] = 1024;
	settings.gain[GAIN_SQUARE] = 1024;
	settings.gain[GAIN_SINE] = 1024;
	settings.gain[GAIN_TRIANGLE] = 1024;
	settings.gain[GAIN_SAWTOOTHDOWN] = 1024;
	settings.gain[GAIN_SAWTOOTHUP] = 1024;
	settings.gain[GAIN_SPRING] = 1024;
	settings.gain[GAIN_DAMPER] = 512;	// 100%だとゴリゴリする
	settings.gain[GAIN_INERTIA] = 1024;
	settings.gain[GAIN_FRICTION] = 1024;
	settings.gain[GAIN_ENDSTOP] = 1024;

	settings.centerButton = -1; // no center button

	settings.debounce = 10;	// SWのチャタリング除去をしないなら0
	settings.minForce = 0;
	settings.maxForce = 16383;
	settings.cutForce = 16383;

	settings.endstopOffset = DEFAULT_ENDSTOP_OFFSET;
	settings.endstopWidth = DEFAULT_ENDSTOP_WIDTH;
  
	for (i = 0; i < 7; i++)
	{
		wheel.analogAxes[i]->setLimits(0, 1023);
		wheel.analogAxes[i]->setCenter(-32768);	// no center
		if (!wheel.analogAxes[i]->autoCenter)
		{
		   wheel.analogAxes[i]->setDZ(0);
		}

		wheel.analogAxes[i]->bitTrim = 0;
		wheel.analogAxes[i]->outputDisabled = 0;
	}
	wheel.axisWheel->setRange(WHEEL_RANGE_DEFAULT);
	wheel.ffbEngine.maxVelocityDamperC = 16384.0 / DEFAULT_MAX_VELOCITY;
	wheel.ffbEngine.maxVelocityFrictionC = 16384.0 / DEFAULT_MAX_VELOCITY;
	wheel.ffbEngine.maxAccelerationInertiaC = 16384.0 / 
											DEFAULT_MAX_ACCELERATION;
}
/*** end of "MyAFFBWheel.ino" ***/
