#include "stdarg.h"
#include "zf_common_headfile.h"
#include "ptf.h"


union cvt                             
{
	float a[100];
	char b[400];
};
union cvt cvt;

int len;
int n=0;
int ptf(char* format,...)
{
	va_list ap;
	va_start(ap,format);
	char*p=format;
	len=strlen(format);
	for(int k=0;k<len;k++)
	{
		if(*p!='%')
		{
			wireless_uart_send_byte(*p);
			p++;
		}
		else
		{
			k++;
			switch(*(++p))
			{
				case'd':
					cvt.a[n]=(float)va_arg(ap,int);
					n++;
					p++;
					break;
				case'f':
					cvt.a[n]=(float)va_arg(ap,double);
					n++;
					p++;
					break;
				case'l':
					switch(*(++p))
					{
						case'd':
							cvt.a[n]=(float)va_arg(ap,long);
							n++;
							p++;
							k++;
							break;
						case'f':
							cvt.a[n]=(float)va_arg(ap,double);
							n++;
							p++;
							k++;
							break;
					}
					break;
			}
		}
	}
	for(int i=0;i<n*4;i++)
	{
		wireless_uart_send_byte(cvt.b[i]);
	}
	n=0;
	wireless_uart_send_byte(0x00);
	wireless_uart_send_byte(0x00);
	wireless_uart_send_byte(0x80);
	wireless_uart_send_byte(0x7f);
	
	va_end(ap);
	return 0;
}