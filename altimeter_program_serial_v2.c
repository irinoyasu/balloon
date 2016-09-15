/*****************************/
/* シリアル通信およびI2C通信 */
/*****************************/
/*---------------------------
変更履歴
v1 : 初版作成(未動作)
-----------------------------*/
#include<wiringPi.h>
#include<wiringSerial.h>
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<time.h>
#include<string.h>

//#define VIDEO_FILE ". ./video_record_10sec.sh &"//ビデオ実行ファイル
//#define VIDEO_FILE_CYCLE 12//ビデオ再録画間隔[s]
#define VIDEO_FILE ". ./video_record_10min.sh &"//ビデオ実行ファイル
#define VIDEO_FILE_CYCLE 603//ビデオ再録画間隔[s]

/* デバッグ用関数 */
//#define debug
//#define gps_debug
//#define alt_debug
//#define telemetry_debug
//#define video_off

/* MS5607用変数，アドレスおよびコマンド */
//ファイル変数
int fd_Altimeter;//Altimeter
int fd_Telemetry;//Telemetry
int fd_GPS;//GPS
//受信データ
char Altimeter_Data[256];//Altimeter
char Telemetry_Data[256] = "ERROR\r\n";//MU-1N
char GPS_Data[256];//XTEND

/* 各種センサデータ */
double Altitude = 0.0;
double Latitude = 0.0;
double Longtitude = 0.0;
double Temperature = 0.0;
double Pressure = 0.0;
int MU1N_count = 0;
int Altimeter_count = 0;
int GPS_count = 0;
char ADC1[20]="";
char ADC2[20]="";



/* 関数一覧 */
void Serial_Init();
int Recieve_Altimeter_Data();
int MU1N_Receive();
void XTEND_Trans();
void GPS_Coordinate(int max_count);
int GPS_Receive();
void Altitude_Request();
void Altimeter_Coordinate(int max_count);

/* メイン関数 */
int main(void) {
	//ローカル変数群
	struct tm* local_date;//現在時刻
	struct tm* start_date;//現在時刻
	char filename[256];//出力ファイル名
	FILE* fp;//出力ファイルポインタ

	time_t timer;
	time_t section_time;
	time_t now_time;
	time_t measure_time;
	int camera_flag = 0;
	int serial_recieve_count;
	int i;


	//スタート宣言
	printf("Program Start\n");
	//シリアルを初期化
	Serial_Init();
	//現在時刻を取得
	time(&timer);
	//local_date = localtime(&timer);
	start_date = localtime(&timer);
	//出力ファイル名取得
	sprintf(filename, "data_log_%04d%02d%02d%02d%02d%02d.csv", start_date->tm_year + 1900, start_date->tm_mon + 1, start_date->tm_mday, start_date->tm_hour, start_date->tm_min, start_date->tm_sec);
	//ログファイル作成
	fp = fopen(filename, "w");
	if (fp == NULL)
	{
		printf("ファイルが開けません\n");
		return -1;
	}
	fprintf(fp, "Time[s],Pressure[hPa],Temp[deg],Altitude[m],ADC1[-],ADC2[-],Lat,Lon,MU1N\r\n");//ログヘッダー
	fclose(fp);
	time(&measure_time);
	//メインループ
	while (1)
	{
		//現在時間を更新
		time(&now_time);

		//1秒ごとに高度データリクエストおよびログデータ出力処理
		if ((int)(now_time - measure_time) >= 1)
		{
			//気圧高度計処理
			Altitude_Request();//データリクエスト
			time(&measure_time);
			//ファイル出力
			fp = fopen(filename, "a");
			if (fp != NULL)
			{
				fprintf(fp, "%d,%.2f,%.2f,%.2f,%s,%s,%.5f,%.5f,%s", (int)(now_time - timer), Pressure, Temperature, Altitude, ADC1, ADC2, Latitude, Longtitude, Telemetry_Data);
#ifndef debug
				printf("%d,%.2f,%.2f,%.2f,%s,%s,%.5f,%.5f,%s", (int)(now_time - timer), Pressure, Temperature, Altitude, ADC1, ADC2, Latitude, Longtitude, Telemetry_Data);
#endif
				fclose(fp);
				
			}
			//テレメトリー送信
			XTEND_Trans();
		}
		//高度データ受信
		Altimeter_count = Recieve_Altimeter_Data();
		if (Altimeter_count > 0)Altimeter_Coordinate(Altimeter_count);
		//テレメトリーデータ受信
		MU1N_count = MU1N_Receive();//データ受信
		//GPSデータ受信
		GPS_count = GPS_Receive();
		if (GPS_count > 0)GPS_Coordinate(GPS_count);

		//カメラ処理
		//10分ごとに動画新規ファイル作成,4時間後以降は1snapshot/1min，5時間後は全ての計測終了
#ifndef video_off
		if (camera_flag < 24)
		{
			if (camera_flag == 0)
			{
				time(&section_time);
				system(VIDEO_FILE);
				printf("Video Record Start:%d\n", camera_flag);
				camera_flag++;
			}
			else
			{
				if ((int)(now_time - section_time) > VIDEO_FILE_CYCLE)
				{
					time(&section_time);
					system(VIDEO_FILE);
					printf("Video Record Start:%d\n", camera_flag);
					camera_flag++;
				}
			}
		}
		else
		{
			//1分間隔
			if (camera_flag == 24)
			{
				system(". ./snapshot.sh &");
				printf("SnapShot:%d\n", camera_flag);
				time(&section_time);
				camera_flag++;
			}
			else if ((int)(now_time - section_time) > 60)
			{
				if (camera_flag < 86)
				{
					system(". ./snapshot.sh &");
					printf("SnapShot:%d\n", camera_flag);
					time(&section_time);
					camera_flag++;
				}
				else
				{
					//システムシャットダウン？
					//system("sudo shutdown -h now");
					printf("Program Finish\n");
					return 0;
				}
			}
		}
#endif
	}
	return 0;
}

/*----------シリアル通信用関数群--------------*/
/* シリアルポート初期化関数 */
void Serial_Init()
{
	//Altimeter Serial Open
	fd_Altimeter = serialOpen("/dev/ttyUSB0", 9600);
	if (fd_Altimeter < 0)
	{
		printf("Can not open serial0 port\n");
		while (1);
	}
	//Telemetry Serial Open
	fd_Telemetry = serialOpen("/dev/ttyUSB1", 9600);
	if (fd_Telemetry < 0)
	{
		printf("Can not open serial1 port\n");
		while (1);
	}
	//GPS Serial Open
	fd_GPS = serialOpen("/dev/ttyUSB2", 9600);
	if (fd_GPS < 0)
	{
		printf("Can not open serial2 port\n");
		while (1);
	}
	printf("Serial Initialaize Finish\r\n");
}

/* シリアルで高度データ受信 */
int Recieve_Altimeter_Data()
{
	static int r_count=0;//受信数
	int return_count;//返り値
	int i;
	while(serialDataAvail(fd_Altimeter))
	{
		Altimeter_Data[r_count] = serialGetchar(fd_Altimeter);//データ受信
//#ifdef alt_debug
//		printf("%c", Altimeter_Data[r_count]);//1文字ずつの表示
//#endif
		if (r_count > 1)
		{
			//改行コードが送られたらデータ処理にかける
			if (Altimeter_Data[r_count-1] == 0x0d && Altimeter_Data[r_count] == 0x0a)
			{
#ifdef alt_debug
				for (i = 0; i < r_count+1; i++)
				{
					printf("%c", Altimeter_Data[i]);//改行ごとに表示
				}
#endif
				return_count = r_count;
				r_count = 0;
				return return_count;
				break;
			}
		}
		if (r_count == 256)
		{
			r_count = 0;
			break;//受信許容を越えるので一度受信をやめる
		}
		else
		{
			r_count++;
		}
	}
	return -r_count;
}

/* シリアルでテレメトリーデータ受信 */
int MU1N_Receive()
{
	static int r_count;//受信数
	static char MU1N_Data[256];//受信データ
	int return_count;//返り値
	while (serialDataAvail(fd_Telemetry))
	{
		MU1N_Data[r_count] = serialGetchar(fd_Telemetry);//データ受信
////#ifdef telemetry_debug
////		printf("%c", Telemetry_Data[r_count]);//受信したデータを表示
////#endif
		if (r_count > 1)
		{
			//改行コードが送られたらデータ処理にかける
			if (MU1N_Data[r_count - 1] == 0x0d && MU1N_Data[r_count] == 0x0a)
			{
				return_count = r_count;
				sprintf(Telemetry_Data, "%s", MU1N_Data);
				r_count = 0;
#ifdef telemetry_debug
				printf("RECIEVE:%s", MU1N_Data);
#endif
				memset(MU1N_Data, '\0', 256);
				return return_count;
				break;
			}
		}
		if (r_count == 256)
		{
			r_count = 0;
			break;//受信許容を越えるので一度受信をやめる
		}
		else
		{
			r_count++;
		}
	}
	return -r_count;//受信数が正の時，受信完了，その他の時受信中
}

/* シリアルでテレメトリーデータ送信 */
void XTEND_Trans()
{
	char Trans_Data[256];
	char MU1N_Edit[256];
	int i;
	sprintf(Trans_Data, "%.5f,%.5f,%.2f,%s",Latitude,Longtitude,Altitude, Telemetry_Data);
	serialPuts(fd_Telemetry, Trans_Data);	
#ifdef telemetry_debug
	printf("%.5f,%.5f,%.2f,%s", Latitude, Longtitude, Altitude, Telemetry_Data);
#endif
}

/* GPSデータ処理 */
void GPS_Coordinate(int max_count)
{
	//例：$GPGGA,085120.307,3541.1493,N,13945.3994,E,1,08,1.0,6.9,M,35.9,M,,0000*5E
	//例：$GPGGA,000001.000,,,,,,,.,,,,,,0000*5E
	//    0123456789012345678901234567890123456789012345678901234567890123456789012
	//    0         1         2         3         4         5         6         7
	int i = 0, j = 0;//汎用変数
	int now_count = 0;
	char gps_lat[20];//LATのASCIIデータ格納予定
	char gps_lon[20];//LONのASCIIデータ格納予定
	if (max_count > 0)
	{
		if (GPS_Data[0] == 0x24)//先頭文字が正常化確認
		{
			//GPGGAをさがす
			if (GPS_Data[3] == 0x47 && GPS_Data[4] == 0x47)
			{
				//GPGGAなので，２番目と４番目のGPSデータを抽出
				//UTCの区切り(,)を探す，ただしGPGGAの部分は省略
				for (i = 7; i < max_count; i++)
				{
					if (GPS_Data[i] == 0x2C)
					{
						now_count = i;//見つけたカンマの位置を記録
						break;
					}
				}
				if (i == max_count)return;//データがエラーなのでこれ以降処理しない
				//次の,を探す，ただしGPGGAの部分は省略(LAT)
				for (i = now_count + 1; i < max_count; i++)
				{
					if (GPS_Data[i] == 0x2C)
					{
						if (i - now_count == 1)
						{
							//データがないので0を代入
							sprintf(gps_lat,"0");
						}
						else
						{
							for (j = 0; j < i - now_count; j++)
							{
								gps_lat[j] = GPS_Data[now_count + 1 + j];
							}
						}
						now_count = i;
						break;
					}
				}
				if (i == max_count)return;//データがエラーなのでこれ以降処理しない
				//次の,を探す，ただしGPGGAの部分は省略(North or South)
				for (i = now_count + 1; i < max_count; i++)
				{
					if (GPS_Data[i] == 0x2C)
					{
						now_count = i;
						break;
					}
				}
				if (i == max_count)return;//データがエラーなのでこれ以降処理しない
				//次の,を探す，ただしGPGGAの部分は省略(LON)
				for (i = now_count + 1; i < max_count; i++)
				{
					if (GPS_Data[i] ==0x2C)
					{
						if (i - now_count == 1)
						{
							//データがないので0を代入
							sprintf(gps_lon,"0");
						}
						else
						{
							for (j = 0; j < i - now_count; j++)
							{
								gps_lon[j] = GPS_Data[now_count + 1 + j];
							}
						}
						now_count = i;
						break;
					}
				}
				Latitude = atof(gps_lat);
				Longtitude = atof(gps_lon);
#ifdef gps_debug
				printf("LAT:%.5f,LON:%.5f\r\n", Latitude, Longtitude);
#endif
			}
		}
		else
		{
			//GPSデータが正常でない
			return;
		}
	}
}

/* シリアルでGPSデータ受信 */
int GPS_Receive()
{
	static int r_count;//受信数
	int return_count;//返り値
	while (serialDataAvail(fd_GPS))
	{
		GPS_Data[r_count] = serialGetchar(fd_GPS);//データ受信
#ifdef gps_debug
		printf("%c", GPS_Data[r_count]);//受信したデータを表示
#endif
		if (r_count > 1)
		{
			if (GPS_Data[r_count - 1] == 0x0d && GPS_Data[r_count] == 0x0a)
			{
				return_count = r_count;
				r_count = 0;
				return return_count;
				break;
			}
		}
		
		if (r_count == 256) {
			r_count = 0;
			break;//受信許容を越えるので一度受信をやめる
		}
		else
		{
			r_count++;
		}
	}
	return -r_count;
}	

/* シリアルで高度データのリクエスト送信 */
void Altitude_Request()
{
	serialPuts(fd_Altimeter, "557");
}

/* 高度データの加工 */
void Altimeter_Coordinate(int max_count)
{
	int i, j;
	int now_count;
	long long int adc1_int;
	long long int adc2_int;
	long long int C_[6] = { 44294, 39431, 26604, 24096, 31503, 27302 }; //初期値
	long long int dT, TEMP = 0;
	long long int OFF, SENS,P;
	double t0 = 30.0;
	double P0 = 1013.25;

	//データ取り出し
	if (max_count > 0)
	{
		//カンマを探す
		//Press
		for (i = 0; i < max_count; i++)if (Altimeter_Data[i] == 0x2C)break;
		if (i == max_count)return;
		//Temp
		now_count = i;
		for (i = now_count + 1; i < max_count; i++)if (Altimeter_Data[i] == 0x2C)break;
		if (i == max_count)return;
		//Altitude
		now_count = i;
		for (i = now_count + 1; i < max_count; i++)if (Altimeter_Data[i] == 0x2C)break;
		if (i == max_count)return;
		//ADC1
		now_count = i;
		for (i = now_count + 1; i < max_count; i++)
		{
			if (Altimeter_Data[i] == 0x2C)
			{
				for (j = 0; j < i - now_count-1; j++)
				{
					ADC1[j] = Altimeter_Data[now_count + 1 + j];
				}
				break;
			}
		}
		if (i == max_count)return;
		//ADC2
		now_count = i;
		for (i = now_count + 1; i < max_count; i++)
		{
			if (Altimeter_Data[i] == 0x0D)
			{
				for (j = 0; j < i - now_count-1; j++)
				{
					ADC2[j] = Altimeter_Data[now_count + 1 + j];

				}
				break;
			}
		}

		//物理量に変換
		adc1_int = atoi(ADC1);
	//	adc1_int = user_atol(ADC1);
		adc2_int = atoi(ADC2);//atol?

#ifdef alt_debug
		printf("adc1:%d%d\r\n", adc1_int);
		printf("adc2:%d%d\r\n", adc2_int);
#endif
#ifdef alt_debug
		printf("ADC1:%s,ADC2:%s\r\n", ADC1, ADC2);
#endif
		dT = adc2_int - C_[4] * pow(2, 8);
		TEMP = 2000 + dT * C_[5] / pow(2, 23);
		OFF = C_[1] * pow(2,17) + dT*C_[3] / pow(2, 6);
		SENS = C_[0] * pow(2,16) + dT*C_[2] / pow(2, 7);
		P = (adc1_int*SENS / pow(2,21) - OFF) / pow(2,15);
		Pressure = (double)P / 100.0;
		Temperature = (double)TEMP / 100.0;
		Altitude = 153.846*(Temperature + 273.15)*(pow((P0 / Pressure), 0.19022256) - 1);
#ifdef alt_debug
		printf("T:%.2f,P:%.2f,H:%.2f\r\n", Temperature, Pressure, Altitude);
#endif
	}
}