#include "XPT2046.h" 
#include "tim.h"

_m_tp_dev tp_dev=
{
	TP_Init,
	TP_Scan,
	TP_Adjust,
	0,
	0, 
	0,
	0,
	0,
	0,	  	 		
	0,
	0,	  	 		
};			


//Ĭ��Ϊtouchtype=0������.
uint8_t CMD_RDX=0XD0;
uint8_t CMD_RDY=0X90;

uint8_t LCD_DIR_Mode;

//SPIд����
//������ICд��1byte����    
//num:Ҫд�������
void TP_Write_Byte(uint8_t num)    
{  
	uint8_t count=0;   
	for(count=0;count<8;count++)  
	{ 	  
		if(num&0x80){TDIN(GPIO_PIN_SET);}
		else {TDIN(GPIO_PIN_RESET);}   
		num<<=1;    
		TCLK(GPIO_PIN_RESET);
		delay_us(1);
		TCLK(GPIO_PIN_SET);		//��������Ч	        
	}		 			    
} 

//SPI������ 
//�Ӵ�����IC��ȡadcֵ
//CMD:ָ��
//����ֵ:����������	   
uint16_t TP_Read_AD(uint8_t CMD)	  
{ 	 
	uint8_t count=0; 	  
	uint16_t Num=0; 
	TCLK(GPIO_PIN_RESET);		//������ʱ�� 	 
	TDIN(GPIO_PIN_RESET); 	//����������
	TCS(GPIO_PIN_RESET); 		//ѡ�д�����IC
	TP_Write_Byte(CMD);//����������
	delay_us(6);//ADS7846��ת��ʱ���Ϊ6us
	TCLK(GPIO_PIN_RESET); 	     	    
	delay_us(1);    	   
	TCLK(GPIO_PIN_SET);		//��1��ʱ�ӣ����BUSY
	delay_us(1);    
	TCLK(GPIO_PIN_RESET); 	
	for(count=0;count<16;count++)//����16λ����,ֻ�и�12λ��Ч 
	{ 				  
		Num<<=1; 	 
		TCLK(GPIO_PIN_RESET);	//�½�����Ч  	    	   
		delay_us(1);    
 		TCLK(GPIO_PIN_SET);
 		if(DOUT)Num++; 		 
	}  	
	Num>>=4;   	//ֻ�и�12λ��Ч.
	TCS(GPIO_PIN_SET);		//�ͷ�Ƭѡ	 
	return(Num);   
}

//��ȡһ������ֵ(x����y)
//������ȡREAD_TIMES������,����Щ������������,
//Ȼ��ȥ����ͺ����LOST_VAL����,ȡƽ��ֵ 
//xy:ָ�CMD_RDX/CMD_RDY��
//����ֵ:����������
#define READ_TIMES 5 	//��ȡ����
#define LOST_VAL 1	  	//����ֵ
uint16_t TP_Read_XOY(uint8_t xy)
{
	uint16_t i, j;
	uint16_t buf[READ_TIMES];
	uint16_t sum=0;
	uint16_t temp;
	for(i=0;i<READ_TIMES;i++)buf[i]=TP_Read_AD(xy);		 		    
	for(i=0;i<READ_TIMES-1; i++)//����
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//��������
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}	
	sum=0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)sum+=buf[i];
	temp=sum/(READ_TIMES-2*LOST_VAL);
	return temp;   
} 

//��ȡx,y����
//��Сֵ��������100.
//x,y:��ȡ��������ֵ
//����ֵ:0,ʧ��;1,�ɹ���
uint8_t TP_Read_XY(uint16_t *x,uint16_t *y)
{
	uint16_t xtemp,ytemp;			// 	Voltage value 		  
	xtemp=TP_Read_XOY(CMD_RDX);
	ytemp=TP_Read_XOY(CMD_RDY);	  
	//if(xtemp<100||ytemp<100)return 0;//����ʧ��
	*x=xtemp;
	*y=ytemp;
	return 1;//�����ɹ�
}

//����2�ζ�ȡ������IC,�������ε�ƫ��ܳ���
//ERR_RANGE,��������,����Ϊ������ȷ,�����������.	   
//�ú����ܴ�����׼ȷ��
//x,y:��ȡ��������ֵ
//����ֵ:0,ʧ��;1,�ɹ���

#define ERR_RANGE 50 //��Χ 

uint8_t TP_Read_XY2(uint16_t *x,uint16_t *y) 
{
		uint16_t x1,y1;
		uint16_t x2,y2;
		uint8_t flag;    
    flag=TP_Read_XY(&x1,&y1);   
    if(flag==0)return(0);
    flag=TP_Read_XY(&x2,&y2);	   
    if(flag==0)return(0);   
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))&&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))//ǰ�����β�����+-50��
    {
        *x=(x1+x2)/2;
        *y=(y1+y2)/2;
        return 1;
    }
		else return 0;	  
}  

//////////////////////////////////////////////////////////////////////////////////		  
//��LCD�����йصĺ���  
//��һ��������
//����У׼�õ�
//x,y:����
//color:��ɫ
void TP_Drow_Touch_Point(uint16_t x,uint16_t y,uint16_t color)
{
	POINT_COLOR=color;
	LCD_DrawLine(x-12,y,x+13,y);//����
	LCD_DrawLine(x,y-12,x,y+13);//����
	LCD_DrawPoint(x+1,y+1);
	LCD_DrawPoint(x-1,y+1);
	LCD_DrawPoint(x+1,y-1);
	LCD_DrawPoint(x-1,y-1);
	LCD_Draw_Circle(x,y,6);//������Ȧ
}	
  
//��һ�����(2*2�ĵ�)		   
//x,y:����
//color:��ɫ
void TP_Draw_Big_Point(uint16_t x,uint16_t y,uint16_t color)
{	    
	POINT_COLOR=color;
	LCD_DrawPoint(x,y);//���ĵ� 
	LCD_DrawPoint(x+1,y);
	LCD_DrawPoint(x,y+1);
	LCD_DrawPoint(x+1,y+1);	 	  	
}
						  
//////////////////////////////////////////////////////////////////////////////////		  
//��������ɨ��
//tp:0,��Ļ����;1,��������(У׼�����ⳡ����)
//����ֵ:��ǰ����״̬.
//0,�����޴���;1,�����д���
uint8_t TP_Scan(uint8_t tp)
{			   
	if(PEN==0)//�а�������
	{
		if(tp)TP_Read_XY2(&tp_dev.x[0],&tp_dev.y[0]);//��ȡ��������
		else//��ȡ��Ļ����
		if(TP_Read_XY2(&tp_dev.x[0],&tp_dev.y[0]))//��ȡ��Ļ����
		{
	 		tp_dev.x[0]=tp_dev.xfac*tp_dev.x[0]+tp_dev.xoff;//�����ת��Ϊ��Ļ����
			tp_dev.y[0]=tp_dev.yfac*tp_dev.y[0]+tp_dev.yoff;  
	 	} 
		if((tp_dev.sta&TP_PRES_DOWN)==0)//֮ǰû�б�����
		{		 
			tp_dev.sta=TP_PRES_DOWN|TP_CATH_PRES;//��������  
			tp_dev.x[4]=tp_dev.x[0];//��¼��һ�ΰ���ʱ������
			tp_dev.y[4]=tp_dev.y[0];  	   			 
		}		
		//while(PEN==0);
	}
	else
	{
		if(tp_dev.sta&TP_PRES_DOWN)//֮ǰ�Ǳ����µ�
		{
			tp_dev.sta&=~(1<<7);//��ǰ����ɿ�	
		}
		else//֮ǰ��û�б�����
		{
			tp_dev.x[4]=0;
			tp_dev.y[4]=0;
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
		}	    
	}
	return tp_dev.sta&TP_PRES_DOWN;//���ص�ǰ�Ĵ���״̬
}
	  
//////////////////////////////////////////////////////////////////////////	 
//������ W25Q16 ����ĵ�ַ�����ַ,ռ��14���ֽ�(RANGE:SAVE_ADDR_BASE~SAVE_ADDR_BASE+13)

#define SAVE_ADDR_BASE 40
//����һ���ṹ��
union TData
{
   int32_t B32_temp;
   int8_t  B8_Temp[4];
} TData; 
 /**********************************��ʼ��������Ϊ0x0����0xFF**********************************/ 
void Init_Buffer (unsigned char *P,unsigned int Count,unsigned char Type) 

{
   unsigned int i;
   if(Type==0)
   {
     for(i=0;i<Count;i++)*P++=0x0;
   }
   else
   {
     for(i=0;i<Count;i++)*P++=0xFF;
   }
}

//����У׼����										    
void TP_Save_Adjdata(void)
{ 
  	uint8_t Flash_Temp[24];
    //����У�����!
    TData.B32_temp = tp_dev.xfac * 100000000; //����xУ������
	memcpy(Flash_Temp,TData.B8_Temp,4);
	TData.B32_temp = tp_dev.yfac * 100000000; //����yУ������
    memcpy(&Flash_Temp[4],TData.B8_Temp,4);	
    //����xƫ����
	TData.B32_temp=tp_dev.xoff;//����xƫ����
	memcpy(&Flash_Temp[4*2],TData.B8_Temp,4);
    //����yƫ����
    TData.B32_temp=tp_dev.yoff;//����yƫ����
	memcpy(&Flash_Temp[4*3],TData.B8_Temp,4);	
	tp_dev.touchtype=LCD_DIR_Mode;  //��Ļ��������
    //���津������
    Flash_Temp[4*4+1]=tp_dev.touchtype; //���津������	
    Flash_Temp[4*4+2]=0X0A;//���У׼����	
	W25QXX_Write((uint8_t*)Flash_Temp,0,24);		//�ӵ�0����ַ����ʼ,�洢У׼����
}


//��ȡ ������ W25Q16 �����У׼ֵ
//����ֵ��1���ɹ���ȡ����
//        0����ȡʧ�ܣ�Ҫ����У׼
uint8_t TP_Get_Adjdata(void)
{					  
	int32_t tempfac;
	int32_t Ttype;
	uint8_t Flash_Temp[24];
	W25QXX_Read(Flash_Temp,0,24);	//�ӵ�0����ַ����ʼ,����У׼����
	Ttype=Flash_Temp[4*4+1]; //��ȡ�����,��Ļ����͵�ǰ��¼�Ĳ��������Ƿ�һ��
	tempfac = Flash_Temp[4*4+2]; //��ȡ�����,���Ƿ�У׼����
	if((tempfac==0X0A)&(Ttype==LCD_DIR_Mode))//�������Ѿ�У׼����			   
 	{ 
				memcpy(TData.B8_Temp,Flash_Temp,4);
			  tempfac=TData.B32_temp;
        tp_dev.xfac = (float)tempfac / 100000000; //�õ�xУ׼����
       	memcpy(TData.B8_Temp,&Flash_Temp[4],4);
			  tempfac=TData.B32_temp;
			  tp_dev.yfac = (float)tempfac / 100000000; //�õ�yУ׼����
        //�õ�xƫ����
        memcpy(TData.B8_Temp,&Flash_Temp[4*2],4);
			  tempfac=TData.B32_temp;
			  tp_dev.xoff=tempfac;
        //�õ�yƫ����
			  memcpy(TData.B8_Temp,&Flash_Temp[4*3],4);
			  tempfac=TData.B32_temp;
			  tp_dev.yoff=tempfac;
		    if(Ttype==0||Ttype==1)tp_dev.touchtype=0;//Flash_Temp[4*4+1];  //��ȡ��������--�������ͣ����ô�������
				else tp_dev.touchtype=1;
				if(tp_dev.touchtype)//X,Y��������Ļ�෴
				{
					CMD_RDX=0X90;
					CMD_RDY=0XD0;	 
				}
				else				   //X,Y��������Ļ��ͬ
				{
					CMD_RDX=0XD0;
					CMD_RDY=0X90;	 
				}		 
				return 1;	 
			}
			return 0;//δУ׼�����߲�������ȷ���򷵻�0
}
//��ʾУ׼���(��������)
//������У׼����
//�õ��ĸ�У׼����
void TP_Adjust(void)
{								 
	uint16_t pos_temp[4][2];//���껺��ֵ
	uint8_t  cnt=0;	
	uint16_t d1,d2;
	uint32_t tem1,tem2;
	double fac; 	
	uint16_t outtime=0;
 	cnt=0;			
	tp_dev.sta=0;//���������ź� 
	tp_dev.xfac=0;//xfac��������Ƿ�У׼��,����У׼֮ǰ�������!�������	 
	while(1)//�������10����û�а���,���Զ��˳�
	{
		tp_dev.scan(1);//ɨ����������
		
		if((tp_dev.sta&0xc0)==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
		{	
			outtime=0;		
			tp_dev.sta&=~(1<<6);//��ǰ����Ѿ����������.		   
			pos_temp[cnt][0]=tp_dev.x[0];
			pos_temp[cnt][1]=tp_dev.y[0];		
			cnt++;	  
			switch(cnt)
			{			   
				case 1:						 
					TP_Drow_Touch_Point(20,20,WHITE);				//�����1 
					TP_Drow_Touch_Point(lcddev.width-20,20,RED);	//����2
					break;
				case 2:
 					TP_Drow_Touch_Point(lcddev.width-20,20,WHITE);	//�����2
					TP_Drow_Touch_Point(20,lcddev.height-20,RED);	//����3
					break;
				case 3:
 					TP_Drow_Touch_Point(20,lcddev.height-20,WHITE);			//�����3
 					TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,RED);	//����4
					break;
				case 4:	 //ȫ���ĸ����Ѿ��õ�
	    		       //�Ա����
					tem1=abs(pos_temp[0][0]-pos_temp[1][0]);//x1-x2
					tem2=abs(pos_temp[0][1]-pos_temp[1][1]);//y1-y2
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,2�ľ���
					tem1=abs(pos_temp[2][0]-pos_temp[3][0]);//x3-x4
					tem2=abs(pos_temp[2][1]-pos_temp[3][1]);//y3-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�3,4�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05||d1==0||d2==0)//���ϸ�
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
 						continue;
					}
					tem1=abs(pos_temp[0][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[0][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,3�ľ���
					tem1=abs(pos_temp[1][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[1][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�2,4�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//���ϸ�
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
						continue;
					}//��ȷ��		   
					//�Խ������
					tem1=abs(pos_temp[1][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[1][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,4�ľ���
					tem1=abs(pos_temp[0][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[0][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�2,3�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//���ϸ�
					{
						cnt=0;
 				    TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 				TP_Drow_Touch_Point(20,20,RED);								//����1
						continue;
					}//��ȷ��	
					//������
					tp_dev.xfac=(float)(lcddev.width-40)/(pos_temp[1][0]-pos_temp[0][0]);//�õ�xfac		 
					tp_dev.xoff=(lcddev.width-tp_dev.xfac*(pos_temp[1][0]+pos_temp[0][0]))/2;//�õ�xoff
			
					tp_dev.yfac=(float)(lcddev.height-40)/(pos_temp[2][1]-pos_temp[0][1]);//�õ�yfac
					tp_dev.yoff=(lcddev.height-tp_dev.yfac*(pos_temp[2][1]+pos_temp[0][1]))/2;//�õ�yoff  
					
					if(abs(tp_dev.xfac)>2||abs(tp_dev.yfac)>2)//������Ԥ����෴��.
					{
						cnt=0;
 				    TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 				TP_Drow_Touch_Point(20,20,RED);								//����1
						tp_dev.touchtype=!tp_dev.touchtype;//�޸Ĵ�������.
						if(tp_dev.touchtype)//X,Y��������Ļ�෴
						{
							CMD_RDX=0X90;
							CMD_RDY=0XD0;	 
						}
						else				   //X,Y��������Ļ��ͬ
						{
							CMD_RDX=0XD0;
							CMD_RDY=0X90;	 
						}			    
						continue;
					}	
					TP_Save_Adjdata(); //����У׼����
 					LCD_Clear(WHITE);//����   
					return;//У�����				 
			}
		}
		HAL_Delay(10);
		outtime++;
		if(outtime>1000)
		{
			TP_Get_Adjdata();//��ȡУ׼����
			break;
	 	} 
 	}
}	
//��������ʼ��  		    
//����ֵ:0,û�н���У׼
//       1,���й�У׼
uint8_t TP_Init(void)
{	
		TP_Read_XY(&tp_dev.x[0],&tp_dev.y[0]);//��һ�ζ�ȡ��ʼ��
		TP_Read_XY(&tp_dev.x[0],&tp_dev.y[0]);//��һ�ζ�ȡ��ʼ��	 
		if(TP_Get_Adjdata())
		{
			return 0;//�Ѿ�У׼
		}
		else			  		//δУ׼
		{ 										    
			LCD_Clear(WHITE);	//����
			TP_Adjust();  		//��ĻУ׼  
		}	
		TP_Get_Adjdata();	
		return 1; 									 
}


