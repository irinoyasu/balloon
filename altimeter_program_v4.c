/*****************************/
/* シリアル通信およびI2C通信 */
/*****************************/
/*---------------------------
変更履歴
v1 : 初版作成(未動作)
v2 : 大幅修正(動作確認)
v3 : ファイル保存
v4 : ビデオ連動
-----------------------------*/
#include<wiringPi.h>
#include<wiringPiI2C.h>
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<time.h>

#define MS5611_ADDR 0x76 //CBR =HIGHの時は0x76,LOWの時は0x77

/* MS5607用変数，アドレスおよびコマンド */
unsigned char cmd_reset    = 0x1e;
unsigned char cmd_adc_read = 0x00;
unsigned char cmd_adc_conv = 0x40;
unsigned char cmd_adc_d1   = 0x00;
unsigned char cmd_adc_d2   = 0x10;
unsigned char cmd_adc_256  = 0x00;
unsigned char cmd_adc_512  = 0x02;
unsigned char cmd_adc_1024 = 0x04;
unsigned char cmd_adc_2048 = 0x06;
unsigned char cmd_adc_4096 = 0x08;
unsigned char cmd_prom_rd  = 0xa0;
int fd_I2C;
unsigned char DevAddr; // 0x77
unsigned int c0,c1,c2,c3,c4,c5,c6,c7;//16bit
unsigned int d1,d2;//24bit
int dt,temp;
int64_t off,sens;
int p;

/* 関数一覧 */
unsigned int MS5607_swap16(unsigned int reg);
void MS5607_debug_set();
void MS5607_debug();
void MS5607_Init();
void MS5607_Get();
double MS5607_Read(unsigned char data_type);
double getHeight(double hPa);

/* メイン関数 */
int main(void){
	//ローカル変数群
	double altitude;//高度
	double pressure;//気圧
	double tempareture;//温度
	struct tm* local_date;//現在時刻
	struct tm* start_date;//現在時刻
	char filename[256];//出力ファイル名
	FILE* fp;//出力ファイルポインタ
	
	time_t timer;
	time_t section_time;
	time_t now_time;
	int camera_flag = 0;
	
	
	//スタート宣言
	printf("Altimeter and Tempareture Sensor Program\n");
	//現在時刻を取得
	time(&timer);
    //local_date = localtime(&timer);
	start_date = localtime(&timer);
	//出力ファイル名取得
	sprintf(filename,"altimeter_log_%04d%02d%02d%02d%02d%02d.csv",start_date->tm_year+1900,start_date->tm_mon+1,start_date->tm_mday,start_date->tm_hour,start_date->tm_min,start_date->tm_sec);
	//ログファイル作成
	fp = fopen(filename,"w");
	if(fp==NULL)
	{
		printf("ファイルが開けません\n");
		return -1;
	}
	fprintf(fp,"Pressure[hPa],Temp[deg],Altitude[m]\r\n");
	fclose(fp);
	
	//初期化
	MS5607_Init();
	
	while(1)
	{
		MS5607_Get();//データ取得
		pressure = MS5607_Read(1);
		tempareture = MS5607_Read(0);
		altitude = getHeight(pressure);
		//表示
		printf("Pressure:%.2f[hPa],Tempretur:%.2f,Altitude:%.2f\n",pressure,tempareture,altitude);
		//ファイル出力
		fp = fopen(filename,"a");
		if(fp != NULL)
		{
			fprintf(fp,"%.2f,%.2f,%.2f\r\n",pressure,tempareture,altitude);
			fclose(fp);
		}
		//10分ごとに動画新規ファイル作成,4時間後以降は1snapshot/1min，5時間後は全ての計測終了
		time(&now_time);
		if(camera_flag < 24)
		{
			if(camera_flag == 0)
			{
				time(&section_time);
				camera_flag++;
				system(". video_record_10min.sh &");
			}
			else
			{
				if((int)(now_time - section_time)>605)
				{
					time(&section_time);
					camera_flag++;
					system(". video_record_10min.sh &");
				}
			}
		}
		else
		{
			if(camera_flag == 24)
			{
				if((int)(now_time - section_time)>605)
				{
					camera_flag++;
				}
			}
			else
			{
				if((int)(now_time - section_time)>60)
				{
					if(camera_flag < 86)
					{
						camera_flag++;
						system(". snapshot.sh &");
					}
					else
					{
						//システムシャットダウン？
						system("sudo shutdown -h now");
					}
				}
				
			}
		}
			
		
		//1秒待機(厳密ではない)
		sleep(1);
	}
}

/*----------MS5607用関数群--------------*/
/* swap関数 */
unsigned int MS5607_swap16(unsigned int reg)
{
	reg=reg&0x0000ffff;
	reg=(((reg<<8)|(reg>>8))&0x0000ffff);
	return(reg);
}

/* 較正値デバッグ用数値 */
void MS5607_debug_set()
{
	//debug data;
	c1=46372;
	c2=43981;
	c3=29059;
	c4=27842;
	c5=31553;
	c6=28165;
	d1=6465444;
	d2=8077636;
}

/* デバッグ用関数 */
void MS5607_debug()
{
	printf("c1   c2    c3    c4    c5    c6\n");
	printf("%05d %05d %05d %05d %05d %05d\n",
	c1,c2,c3,c4,c5,c6);

	printf("%04x %016x\n",0,c0);
	printf("%04x %016x\n",1,c1);
	printf("%04x %016x\n",2,c2);
	printf("%04x %016x\n",3,c3);
	printf("%04x %016x\n",4,c4);
	printf("%04x %016x\n",5,c5);
	printf("%04x %016x\n",6,c6);
	printf("%04x %016x\n",7,c7);

	printf("d1        d2\n");
	printf("%08d  %08d\n",d1,d2);

	printf("dt        temp\n");
	printf("%08d  %08d\n",dt,temp);

	printf("off       sens    p\n");
	printf("%10lld  %10lld  %08d\n",off,sens,p);

}

/* MS5607初期化 */
void MS5607_Init()
{
	unsigned int reg;
	//MS5607デバイスアドレス設定
	DevAddr=0x77;//CBR =HIGHの時は0x76,LOWの時は0x77
	//I2C初期化
	fd_I2C= wiringPiI2CSetup(DevAddr);
	if(fd_I2C<0)
	{
		printf("センサー設定エラー\n");
	}

	wiringPiI2CWrite(fd_I2C, cmd_reset); //Reset
	usleep(1000000);
	
	//較正値読み取り
	reg=wiringPiI2CReadReg16(fd_I2C,cmd_prom_rd+0);
	c0=MS5607_swap16(reg);
	reg=wiringPiI2CReadReg16(fd_I2C,cmd_prom_rd+2);
	c1=MS5607_swap16(reg);
	reg=wiringPiI2CReadReg16(fd_I2C,cmd_prom_rd+4);
	c2=MS5607_swap16(reg);
	reg=wiringPiI2CReadReg16(fd_I2C,cmd_prom_rd+6);
	c3=MS5607_swap16(reg);
	reg=wiringPiI2CReadReg16(fd_I2C,cmd_prom_rd+8);
	c4=MS5607_swap16(reg);
	reg=wiringPiI2CReadReg16(fd_I2C,cmd_prom_rd+10);
	c5=MS5607_swap16(reg);
	reg=wiringPiI2CReadReg16(fd_I2C,cmd_prom_rd+12);
	c6=MS5607_swap16(reg);
	reg=wiringPiI2CReadReg16(fd_I2C,cmd_prom_rd+14);
	c7=MS5607_swap16(reg);
}

/* MS5607 値取得 */
void MS5607_Get()
{
	unsigned int reg;
	unsigned char buff[4];
	
	//AD1変換＆読み込み
	wiringPiI2CWrite(fd_I2C, cmd_adc_conv|cmd_adc_d1|cmd_adc_4096); //Convert
	usleep(100000);
	wiringPiI2CWrite(fd_I2C, cmd_adc_read); //
	read(fd_I2C,buff,3);
	reg=0;
	reg=reg |(buff[0]<<16);
	reg=reg |(buff[1]<<8);
	reg=reg |(buff[2]<<0);
	d1=reg; 

	//AD2変換＆読み込み
	wiringPiI2CWrite(fd_I2C, cmd_adc_conv|cmd_adc_d2|cmd_adc_4096); //Convert
	usleep(100000);
	wiringPiI2CWrite(fd_I2C, cmd_adc_read); //
	read(fd_I2C,buff,3);
	reg=0;
	reg=reg |(buff[0]<<16);
	reg=reg |(buff[1]<<8);
	reg=reg |(buff[2]<<0);
	d2=reg; 

	//  debug1();
	
	//物理値へ変換
	dt=(int)(d2-(c5<<8));
	temp=2000+((dt*c6)>>23);
	int64_t x1,x2,x3;
	x1=(int64_t)c2<<17;
	x2=(c4*dt);
	x3=x2>>6;
	off=x1+x3;
	//  printf("x1=%lld x2=%lld x3=%lld off=%lld\n",x1,x2,x3,off);
	//off=(c2<<17)+((c4*dt)>>6);
	sens=(c1<<16)+((c3*dt)>>7);
	x1=d1*sens>>21;
	p=(int)(((int64_t)(x1)-(int64_t)(off))>>15);
	//  debug();
}

/* MS5607 物理値データ取得 */
double MS5607_Read(unsigned char data_type)
{
	if(data_type==0) return ((double)temp/100);
	if(data_type==1) return ((double)p/100);
	return((double)p/100);
}

/* 気圧⇒高度変換 */
double getHeight(double hPa)
{
	//Height = -938.502 * hPa/100.0 + 948697; //t0=30[deg]で1000mまでの線形近似
	double t0 =30.0;double P0 = 1013.25;
	double H;
	H = 153.8*(t0+273.2)*(1-pow((hPa/100.0/P0),0.1902));
	return H;
}